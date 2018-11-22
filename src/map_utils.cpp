#include "map_utils.hpp"

using namespace std;

namespace hhpomdp {
    void MapUtils::init() {
        // generate action remap for arget convex hull transition
        get_action_remap(act_remap, m->convex_trans);
        
        generate_base_reward(base_reward);
        generate_base_transition(base_transition);
        generate_convex_action_set();
        generate_target_transition(ctrans_prob);

        // print_matrix();
    }

    void MapUtils::generate_base_reward(Eigen::MatrixXd& result) {
        /**
         * The reward from the base grid world
         * Currently only the punishment related to invalid
         * actions into the walls and obstacles
         * Positive reward if from the joint state space
         */

        cout << "Generating base reward..." << endl;
        Eigen::MatrixXd r(m->width * m->height, int(ACT_LENGTH));

        FOR_RANGE(x, m->height) {
            FOR_RANGE(y, m->width) {
                int idx_1d = m->get1d_grid(x, y);
                FOR_RANGE(a, ACT_LENGTH) {
                    int next_grid;
                    bool is_valid = get_next_grid(next_grid, idx_1d, a);

                    if(!is_valid) 
                        r(idx_1d, a) = OBS_REWARD;
                    else
                        r(idx_1d, a) = 0;

                } // end for action

            } // end for width
        } // end for height

        result = r;
    }

    void MapUtils::generate_base_transition(Eigen::MatrixXi& result) {
        cout << "Generating base transition..." << endl;
        Eigen::MatrixXi r(m->width * m->height, int(ACT_LENGTH));

        FOR_RANGE(x, m->height) {
            FOR_RANGE(y, m->width) {
                int idx_1d = m->get1d_grid(x, y);
                FOR_RANGE(a, ACT_LENGTH) {
                    int next_grid;
                    bool is_valid = get_next_grid(next_grid, idx_1d, a);

                    if(!is_valid) 
                        r(idx_1d, a) = idx_1d;
                    else
                        r(idx_1d, a) = next_grid;

                } // end for action

            } // end for width
        } // end for height

        result = r;
    }

    void MapUtils::generate_convex_action_set() {
        int cnum = m->cell_num;
        convex_action_set.clear();
        action_mesh.clear();

        int all_act = pow(cnum, robot_num);
        FOR_RANGE(i, all_act) {
            Eigen::VectorXi act = get_act_vec(i, cnum, robot_num);
            convex_action_set.push_back(act);
            vector<int> act_vec(act.size());
            Eigen::VectorXi::Map(&act_vec[0], act.size()) = act;
            action_mesh.push_back(act_vec);
        }

        other_convex_set.clear();

        if(robot_num <= 1) return;

        all_act = pow(cnum, robot_num - 1);
        FOR_RANGE(i, all_act) {
            other_convex_set.push_back(get_act_vec(i, cnum, robot_num - 1));
        }

    }

    void MapUtils::generate_target_transition(Eigen::MatrixXf& result) {
        Eigen::MatrixXf trans = m->convex_trans.cast<float>();
        Eigen::VectorXf c_sum = trans.rowwise().sum();
        trans = trans.array().colwise() / c_sum.array();
        result = trans.transpose();
    }

    Eigen::VectorXi MapUtils::get_act_vec(int v, int cnum, int max_num) {
        Eigen::VectorXi result(max_num);
        int ri = max_num- 1;
        std::div_t dv;
        while(ri >= 0) {
            dv = std::div(v, pow(cnum, ri));
            result(ri) = dv.quot;
            v = dv.rem;
            ri --;
        }
        return result;
    }

    void MapUtils::get_action_remap(MatrixXi& result, MatrixXb& act_map) {
        /**
         * This function generate the remap of actions in the 
         * abstract POMDP level.
         * The main reason is that if we add reward in the top 
         * POMDP level, the reward from MDP might be washed out.
         * (unverified yet)
         * We generate the remap by one-step best action from 
         * the result of djikstra (A* without heuristic).
         */
        cout << "Generating action remap..." << endl;
        int node_size = act_map.rows();
        
        // use the transition matrix as mask
        MatrixXb mask(act_map);
        
        // generate action map for fully connected graph
        Eigen::MatrixXi action_map(node_size, node_size);
        FOR_RANGE(i, node_size) {
            FOR_RANGE(j, node_size)
                action_map(i, j) = j;
        }

        vector<IntPair> unconnected_set;
        vector< vector<int> > neighbor_vec;
        FOR_RANGE(i, node_size) {
            vector<int> n_i;
            FOR_RANGE(j, node_size) {
                if(!mask(i, j)) {
                    unconnected_set.push_back(make_pair(i, j));
                }

                if(mask(i, j) && i != j)
                    n_i.push_back(j);
            }
            neighbor_vec.push_back(n_i);
        }
        
        // do A* on all the blank values
        // here IntPair is used as <node_id, g_score>
        for(auto p : unconnected_set) {
            int start = p.first; 
            int end = p.second;

            vector<int> g_score(node_size);
            fill(g_score.begin(), g_score.end(), 10000);
            g_score[start] = 0;

            vector<IntPair> open_set;
            vector<int> close_set;
            open_set.push_back(make_pair(start, 0));
            make_heap(open_set.begin(), open_set.end(), MinHeapComp());

            vector<int> came_from(node_size);

            while(open_set.size() > 0) {
                IntPair current = open_set.front();
                int cid = current.first;
                if(cid == end)
                    break;
                pop_heap(open_set.begin(), open_set.end());
                open_set.pop_back();
                close_set.push_back(cid);

                // loop through neighbors of current node
                for(auto nid : neighbor_vec[cid]) {
                    if(find(close_set.begin(), close_set.end(), nid) != close_set.end())
                        continue;

                    int tmp_g = g_score[cid] + 1;
                    if(find(open_set.begin(), open_set.end(), make_pair(nid, g_score[nid])) == open_set.end()) {
                        tmp_g = MIN(tmp_g, g_score[nid]);
                        open_set.push_back(make_pair(nid, tmp_g));
                    } else if(tmp_g >= g_score[nid])
                        continue;
                    
                    // best path till now
                    // update came_from based on depth
                    g_score[nid] = tmp_g;
                    if(g_score[nid] == 0) {
                        // this is start, whatever
                        came_from[nid] = -1;
                    } else if(g_score[nid] == 1) {
                        // this is direct neighbor, we want this
                        came_from[nid] = nid;
                    } else {
                        // these are >1 neighbors
                        came_from[nid] = came_from[cid];
                    }

                } // end loop neighbor
                
            } // end while

            // done with A*
            action_map(start, end) = came_from[end];
        } // end for in unconnected_set 

        result = action_map;
    }
    
    void MapUtils::get_convex_from_base1d(vector<int>& result, int base1d) {
        IntPair base2d;
        m->get2d_grid(base2d, base1d);
        result = m->convex_id_map[base2d.first][base2d.second];
    }

    float MapUtils::get_joint_reward(Eigen::VectorXi& robot_state, Eigen::VectorXi& robot_action, int target_state, int target_act) {
        int target_des = base_transition(target_state, target_act);

        int robot_des;
        int reward = 0;
        FOR_RANGE(i, robot_num) {
            // check whether invalid option exist
            robot_des = base_transition(robot_state[i], robot_action[i]);
            reward += base_reward(robot_state[i], robot_action[i]);

            // check whether catch target
            if(robot_des == target_des)
                reward += CATCH_REWARD;
        }

        return reward;
    }

    Eigen::VectorXi MapUtils::get_robot_conv(MatrixXi& r) {
        Eigen::VectorXi r_vec(robot_num);
        FOR_RANGE(i, robot_num) {
            int x = r(i, 0); int y = r(i, 1);
            auto clist = m->convex_id_map[x][y];
            r_vec(i) = clist.front();
        }
        return r_vec;
    }

    bool MapUtils::get_next_grid(int& result, int current, int a) {
        IntPair current2d;
        m->get2d_grid(current2d, current);
        int x = current2d.first; int y = current2d.second;
        if(m->env_map(x, y)) {
            result = current;
            return false;
        }

        assert(a < ACT_LENGTH && a >= 0);
        if(a == NORTH) {
            x--;
        } else if(a == SOUTH) {
            x++;
        } else if(a == WEST) {
            y--;
        } else if(a == EAST) {
            y++;
        }

        // check valid
        if(x < 0 || y < 0) {
            result = current;
            return false;
        }
        else if(x >= m->height || y >= m->height) {
            result = current;
            return false;
        }
        else if(m->env_map(x, y)) {
            result = current;
            return false;
        }
        else {
            result = m->get1d_grid(x, y);
        }
        return true;
    }

    void MapUtils::get_next_pomdp_act(Eigen::VectorXi& result, Eigen::VectorXi& current, Eigen::VectorXi& next) {
        Eigen::VectorXi tmp(robot_num);
        FOR_RANGE(i, robot_num) {
            tmp(i) = act_remap(current(i), next(i));
        }
        result = tmp;
    }

    bool MapUtils::is_visible(int& same_conv, int r_1d, int t_1d) {
        vector<int> r_convex, t_convex;
        get_convex_from_base1d(r_convex, r_1d);
        get_convex_from_base1d(t_convex, t_1d);
        vector<int> v_intersect;
        set_intersection(r_convex.begin(), r_convex.end(),
                        t_convex.begin(), t_convex.end(),
                        back_inserter(v_intersect));
        if(v_intersect.size() > 0) {
            same_conv = v_intersect.front();
            return true;
        }
        return false;
    }

    void MapUtils::print_matrix() {
        cout << "Print matrix in MapUtils with map " << m->id << endl;
        cout << "env map: \n" << m->env_map << endl;
        cout << "convex transition matrix after remap: \n" << act_remap << endl;
        cout << "base mdp reward matrix: \n" << base_reward << endl;
        cout << "base mdp transition matrix: \n" << base_transition << endl;
        cout << "target convex hull transition probabilty: \n" << ctrans_prob << endl;

        cout << "convex action set:\n";
        for(auto a : convex_action_set)
            cout << a.transpose() << endl;

        cout << "action mesh:\n";
        for(auto v : action_mesh) {
            for(auto i : v)
                cout << i << " ";
            cout << endl;
        }

        cout << "other convex set:\n";
        for(auto a : other_convex_set)
            cout << a.transpose() << endl;
    }

};
