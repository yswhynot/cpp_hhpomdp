#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>

#include <Eigen/Core>

#include "cxxopts.hpp"

#include "hhpomdp.hpp"
#include "map.hpp"
#include "type_struct.hpp"

#define RAND_T 0
#define STRA_T 1

using namespace std;
using namespace hhpomdp;
using namespace Eigen;

class Simulation {
public:
    Simulation(shared_ptr<hhpomdp::Map>& input_m, shared_ptr<HHPOMDP>& input_h, bool isave, int tm) : 
        m_ptr(input_m), h_ptr(input_h), is_save(isave), target_move(tm) {}

    MatrixXi action_from_waypoint(MatrixXi& current);
    int find_act_rid(MatrixXi& current, int base);
    MatrixXi get_next(MatrixXi& current, int obs, vector<shared_ptr<State> >& n_ptr);
    int get_next_target(VectorXi& new_loc, VectorXi& cur_loc); // returns action
    int get_obs(const shared_ptr<State>& n_ptr);
    void init(MatrixXi& _robot, VectorXi& _target);
    bool is_catch(VectorXi& prev_t, MatrixXi& prev_r);
    void start(int max_ep);
    void waypoint_from_convex(VectorXi& convex, MatrixXi& current); // change waypoint inside

private:
    shared_ptr<hhpomdp::Map> m_ptr;
    shared_ptr<HHPOMDP> h_ptr;

    int robot_num;
    bool is_first;
    bool is_save;
    int target_move;
    int active_rid;

    MatrixXi robot;     // dim: robot_num * 2, 2d location in grid
    VectorXi target;    // dim: 1 * 2
    MatrixXi init_robot;
    VectorXi init_target;

    MatrixXi waypoint;  // dim: robot_num * 2, 2d location in grid
};

MatrixXi Simulation::action_from_waypoint(MatrixXi& current) {
    MatrixXi result(current);
    MatrixXi diff = waypoint - current;
    MatrixXi dif_sign = diff.array().sign();
    VectorXi min_vec = diff.cwiseAbs().colwise().minCoeff();
    
    // movement is x-major
    FOR_RANGE(i, robot_num) {
        if(dif_sign(i, 0) != 0) {
            float step = float(diff(i, 0)) / float(min_vec(0) + 1);
            step = MAX(1, abs(int(step)));
            result(i, 0) += int(step) * dif_sign(i, 0);
            continue;
        } else if(dif_sign(i, 1) != 0) {
            float step = float(diff(i, 1)) / float(min_vec(1) + 1);
            step = MAX(1, abs(int(step)));
            result(i, 1) += int(step) * dif_sign(i, 1);
        }
    }

    return result;
}

void Simulation::init(MatrixXi& _robot, VectorXi& _target) {
    robot_num = h_ptr->map_util->robot_num;
    robot = _robot;
    init_robot = robot;
    target = _target;
    init_target = target;
    waypoint = MatrixXi::Zero(robot_num, 2);
    is_first = true;
    active_rid = -1;

    srand (time(NULL));
}

int Simulation::find_act_rid(MatrixXi& current, int base_1d) {
    IntPair b2d;
    m_ptr->get2d_grid(b2d, base_1d);
    vector<int> target_conv = m_ptr->convex_id_map[b2d.first][b2d.second];
    FOR_RANGE(i, robot_num) {
        vector<int> robot_conv = m_ptr->convex_id_map[robot(i, 0)][robot(i, 1)];
        vector<int> v_inter;
        set_intersection(robot_conv.begin(), robot_conv.end(),
                target_conv.begin(), target_conv.end(),
                back_inserter(v_inter));
        if(v_inter.size() > 0) {
            return i;
        }
    }
    return -1;
}

MatrixXi Simulation::get_next(MatrixXi& current, int obs, vector<shared_ptr<State> >& n_ptr) {
    cout << "[get_next] current:\n" << current << "\ntarget:\n" << target.transpose() << "\nobs:" << obs << endl;
    n_ptr.front()->print();
    if(n_ptr.front()->is_term)
        return current;

    if(is_first) {
        // always start from pomdp
        auto c_state = n_ptr.front();
        shared_ptr<Edge> e_ptr = c_state->edge_list[obs];
        int opt_a = e_ptr->optimal_act_idx;
        auto child = e_ptr->children[opt_a];
        VectorXi act = e_ptr->action_set[opt_a];
        waypoint_from_convex(act, current);
        is_first = false;
        n_ptr.pop_back();
        n_ptr.push_back(child);
        return action_from_waypoint(current);
    }

    MatrixXi next;
    if(n_ptr.front()->stype == hhpomdp::T_POMDP) {
        if(!current.isApprox(waypoint)) {
            return action_from_waypoint(current);
        }
        cout << "Reach waypoint in pomdp." << endl;

        auto c_state = n_ptr.front();
        // cout << "edge_list:" << endl;
        // for(auto e : c_state->edge_list) {
            // e.second->print();
        // }
        int opt_a = c_state->edge_list[obs]->optimal_act_idx;
        auto child = c_state->edge_list[obs]->children[opt_a];
        if(child->stype != hhpomdp::T_TRANS) {
            VectorXi act = c_state->edge_list[obs]->action_set[opt_a];
            // c_state->edge_list[obs]->print();
            waypoint_from_convex(act, current);
            n_ptr.pop_back();
            n_ptr.push_back(child);
            return action_from_waypoint(current);
        }
        cout << "Going to transition state." << endl;
        n_ptr.pop_back();
        n_ptr.push_back(child);
    }
    if(n_ptr.front()->stype == hhpomdp::T_BASE) {
        auto c_state = n_ptr.front();
        shared_ptr<Edge> e_ptr = c_state->edge_list[obs];
        // cout << "base edge:"; e_ptr->print();
        if(obs >= 0) {
            int opt_a = e_ptr->optimal_act_idx;
            auto child = e_ptr->children[opt_a];
            VectorXi act_vec = c_state->edge_list[obs]->action_set[opt_a];
            int act = act_vec(0);
            assert(active_rid >= 0);
            int r_1d = m_ptr->get1d_grid(robot(active_rid, 0), robot(active_rid, 1));
            int next_r = h_ptr->map_util->base_transition(r_1d, act);
            IntPair next_r2d; m_ptr->get2d_grid(next_r2d, next_r);
            waypoint = current; 
            waypoint(active_rid, 0) = next_r2d.first;
            waypoint(active_rid, 1) = next_r2d.second;
            n_ptr.pop_back();
            n_ptr.push_back(child);
            return waypoint;
        }
        // returning to pomdp states
        else {
            VectorXi robot_conv = h_ptr->map_util->get_robot_conv(current);
            auto child = e_ptr->action_map[robot_conv];
            n_ptr.pop_back();
            n_ptr.push_back(child);
            return get_next(current, m_ptr->cell_num, n_ptr);
        }
    }
    if(n_ptr.front()->stype == hhpomdp::T_TRANS) {
        auto c_state = n_ptr.front();
        int t_1d = m_ptr->get1d_grid(target(0), target(1));
        // active_rid = find_act_rid(current, t_1d);
        // cout << "active_rid:" << active_rid << endl;
        int r_1d = m_ptr->get1d_grid(robot(active_rid, 0), robot(active_rid, 1));
        VectorXi action(1); action << r_1d;
        auto child = c_state->edge_list[t_1d]->action_map[action];
        n_ptr.pop_back();
        n_ptr.push_back(child);
        
        // goint to base state directly
        next = get_next(current, t_1d, n_ptr);
    }
    return next;
}

int Simulation::get_obs(const shared_ptr<State>& n_ptr) {
    int t_1d = m_ptr->get1d_grid(target(0), target(1));
    int conv_num = -1;
    if(n_ptr->stype == T_BASE) {
        int r_1d = m_ptr->get1d_grid(robot(active_rid, 0), robot(active_rid, 1));
        if(h_ptr->map_util->is_visible(conv_num, r_1d, t_1d))
            return t_1d;
        else
            return -1;
    }
    else if(n_ptr->stype == T_POMDP) {
        FOR_RANGE(i, robot_num) {
            int r_1d = m_ptr->get1d_grid(robot(i, 0), robot(i, 1));
            if(h_ptr->map_util->is_visible(conv_num, r_1d, t_1d)) {
                if(conv_num != -1 && conv_num != n_ptr->robot(i))
                    conv_num = -1;
                else {
                    active_rid = i;
                    return conv_num;
                }
            }
        }
        
        // not visible
        return m_ptr->cell_num;
    }
    cout << "ERROR: reach transition node in get_obs" << endl;
    return -1;
}

int get_dis_sum(VectorXi& target, MatrixXi& robot) {
    int num = robot.rows();
    int dis = 0;
    FOR_RANGE(i, num) {
        dis += abs(robot(i, 0) - target(0)) + abs(robot(i, 1) - target(1));
    }
    return dis;
}

int Simulation::get_next_target(VectorXi& new_loc, VectorXi& cur_loc) {
    int act = hhpomdp::NO_ACT;
    int cur_loc_1d = m_ptr->get1d_grid(cur_loc(0), cur_loc(1));
    int new_loc_1d = h_ptr->map_util->base_transition(cur_loc_1d, act);
    
    if(target_move == RAND_T) {
        act = rand() % hhpomdp::ACT_LENGTH;
        new_loc_1d = h_ptr->map_util->base_transition(cur_loc_1d, act);

        if(new_loc_1d == cur_loc_1d)
            act = hhpomdp::NO_ACT;
    } else if(target_move == STRA_T) {
        act = hhpomdp::NO_ACT;

        // get all visible pursuers inside same convex
        vector<int> r_list;
        vector<int> t_conv = m_ptr->convex_id_map[cur_loc[0]][cur_loc[1]];
        FOR_RANGE(ri, robot_num) {
            vector<int> r_conv = m_ptr->convex_id_map[robot(ri, 0)][robot(ri, 1)];
            vector<int> v_inters;
            set_intersection(t_conv.begin(), t_conv.end(), 
                    r_conv.begin(), r_conv.end(), 
                    std::back_inserter(v_inters));
            if(v_inters.size() > 0)
                r_list.push_back(ri);
        }

        if(r_list.size() > 0) {
            MatrixXi vis_robots(r_list.size(), 2);
            vis_robots.setZero();
            FOR_RANGE(i, r_list.size()) {
                vis_robots.row(i) += robot.row(r_list[i]);
            }
            int sum_dis = get_dis_sum(cur_loc, vis_robots);
            int tmp_sum = 0;
            FOR_RANGE(a, hhpomdp::ACT_LENGTH) {
                int tmp_new_loc_1d = h_ptr->map_util->base_transition(cur_loc_1d, a);
                IntPair new_loc_pair;
                m_ptr->get2d_grid(new_loc_pair, tmp_new_loc_1d);
                VectorXi new_vec(2);
                new_vec << new_loc_pair.first, new_loc_pair.second;
                tmp_sum = get_dis_sum(new_vec, vis_robots);

                if(tmp_sum > sum_dis) {
                    act = a;
                    sum_dis = tmp_sum;
                    new_loc_1d = tmp_new_loc_1d;
                }
            } // end for_range
        }
    }
    IntPair new_loc_pair;
    m_ptr->get2d_grid(new_loc_pair, new_loc_1d);
    new_loc(0) = new_loc_pair.first; new_loc(1) = new_loc_pair.second;
    // cout << "target:" << cur_loc.transpose() << ", act:" << act << ", robots:\n" << robot << endl;
    return act;
}

bool Simulation::is_catch(VectorXi& prev_t, MatrixXi& prev_r) {
    FOR_RANGE(i, robot_num) {
        if(robot(i, 0) == target(0) && robot(i, 1) == target(1))
            return true;
        
        // switch place directly
        if(robot(i, 0) == prev_t(0) && robot(i, 1) == prev_t(1)
                && prev_r(i, 0) == target(0) && prev_r(i, 1) == target(1))
            return true;
    }
    return false;
}

void Simulation::start(int max_ep) {
    int ep = 0;
    std::ostringstream ss;
    if(is_save) {
        ofstream file_map;
        ss << "../data/map/map" << m_ptr->id << ".txt";
        file_map.open(ss.str());
        file_map << m_ptr->env_map;
        file_map.close();
    }

    while(ep < max_ep) {
        int step = 0;
        init(init_robot, init_target);
        VectorXi prev_t(target); MatrixXi prev_r(robot);
        vector<shared_ptr<State> > n_ptr;
        n_ptr.push_back(h_ptr->fvi->root);

        ofstream file;
        if(is_save) {
            ss.str("");
            ss << "../data/ep/ep" << ep << "_r" << robot_num << "_m" << m_ptr->id << ".txt";
            file.open (ss.str());
        }
        while(! is_catch(prev_t, prev_r)) {
            prev_t = target; prev_r = robot;
            
            if(is_save)
                file << robot << "\n" << target.transpose() << "\n\n";

            int t_act = get_next_target(target, prev_t);
            if(is_catch(prev_t, prev_r)) break;
            int obs = get_obs(n_ptr.front());
            robot = get_next(robot, obs, n_ptr);
            step ++;
        }

        if(is_save)
            file << robot << "\n" << target.transpose() << "\n\n";
        file.close();
        cout << "caught with step: " << step << endl;
        ep ++;
    }
}

void Simulation::waypoint_from_convex(VectorXi& convex, MatrixXi& current) {
    int max_dis = m_ptr->width * m_ptr->height;
    FOR_RANGE(i, robot_num) {
        vector<IntPair> cell_in_conv = m_ptr->convex_cover[convex(i)];
        int min_dis = max_dis; IntPair min_ip;
        int x = current(i, 0); int y = current(i, 1);
        for(auto ip : cell_in_conv) {
            int manh = abs(ip.first - x) + abs(ip.second - y);
            if(manh < min_dis) {
                min_dis = manh;
                min_ip = ip;
            }
        }
        waypoint(i, 0) = min_ip.first; waypoint(i, 1) = min_ip.second;
    }
    // cout << "[waypoint_from_convex] convex\n" << convex.transpose() << "\ncurrent\n" << current << "\nwaypoint\n" << waypoint << endl;
}

int main(int argc, char *argv[]) {
    cxxopts::Options options("hhpomdp", "Experiment for HHPOMDP");
    options.add_options()
        ("s,save", "Flag of saving the result")
        ("m,map", "Map id", cxxopts::value<int>())
        ("e,ep", "Episode", cxxopts::value<int>())
        ("n,robot-number", "Robot number", cxxopts::value<int>())
        ;
    auto result = options.parse(argc, argv);


    int robot_num = result["robot-number"].as<int>();
    int map_id = result["map"].as<int>();
    int ep = result["ep"].as<int>();
    bool is_save = result["save"].as<bool>();
    cout << "Generating policy for map " << map_id << endl;
    MatrixXi robot;
    VectorXi target(2); 
    if(map_id == 1) {
        robot = MatrixXi::Zero(robot_num, 2);
        target << 0, 3;
    }
    else if(map_id == 4) {
        robot = MatrixXi::Ones(robot_num, 2);
        FOR_RANGE(i, robot_num) {
            robot(i, 0) = i+1;
        }
        VectorXi t(2); t<< 10, 4; target = t;
    } else if(map_id == 6) {
        robot = MatrixXi::Ones(robot_num, 2);
        FOR_RANGE(i, robot_num) {
            robot(i, 0) = 2;
        }
        VectorXi t(2); t<< 2, 3; target = t;
    } else if(map_id == 3) {
        robot = MatrixXi::Zero(robot_num, 2);
        VectorXi t(2); t<< 4, 4; target = t;
    } else {
        cout << "ERROR: invalid map id" << endl;
    }

    shared_ptr<hhpomdp::Map> m(new hhpomdp::Map(map_id));
    shared_ptr<HHPOMDP> h(new HHPOMDP(m, robot_num, robot));
    bool flag = h->start();

    Simulation handler(m, h, is_save, RAND_T);
    handler.init(robot, target);
    handler.start(ep);
    return 0;
}
