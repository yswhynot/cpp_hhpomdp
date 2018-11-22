#include "mdp_struct.hpp"

using namespace std;

namespace hhpomdp {
    float round_float(float x) {
        return roundf(x * ROUND_PRE) / ROUND_PRE;
    }

    void format_belief(VectorXf& input) {
        input /= input.sum();
        input = input.unaryExpr(&round_float);
    }

    shared_ptr<State> find_child(const shared_ptr<State>& s, SetList& full_list, SetList& open_list) {
        bool exist = false;
        auto it = full_list.find(s);

        if(it != full_list.end())
            exist = true;

        if(!exist) {
            s->id = full_list.size();
            full_list.insert(s);
            if(!s->has_expand)
                open_list.insert(s);
            return s; 
        }
        return *it;
    }

    void PomdpState::set_belief(VectorXf& input) {
        format_belief(input);
        target = input;
    }

    void PomdpState::add_edge(VectorXf& tmp_target, SetList& full_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu) {
        shared_ptr<Edge> e_ptr = make_shared<Edge>(obs, T_POMDP);
        assert(!isnan(tmp_target(0)));
        
        // the target is visible already
        if(obs != convex_num) {
            // since the target is visible
            // max(tmp_target) should = 1
            // should go to trans
            assert(tmp_target.maxCoeff() == 1.0);
            VectorXi dif = robot;
            dif.array() -= obs;
            dif = dif.cwiseAbs();
            if(dif.minCoeff() != 0)
                return;

            shared_ptr<TransState> trans = make_shared<TransState>();
            VectorXf tmp_t(1); tmp_t << float(obs);
            trans->target = tmp_t;
            VectorXi tmp_r(1); tmp_r << obs;
            trans->robot = tmp_r;

            auto new_child = find_child(trans, full_list, open_list);
            VectorXi action(1); action << GO_TO_TRANS;
            new_child->depth = MIN(new_child->depth, depth+1);
            e_ptr->add_child(new_child, action);
        }

        // target not visible
        else {
            for(auto act : mu->convex_action_set) {
                VectorXi next_act;
                mu->get_next_pomdp_act(next_act, robot, act);

                if(find(e_ptr->action_set.begin(), e_ptr->action_set.end(), next_act) != e_ptr->action_set.end())
                    continue;

                // new action found
                shared_ptr<PomdpState> pom = make_shared<PomdpState>();
                pom->robot = next_act;
                pom->target = tmp_target;
                auto new_pom = find_child(pom, full_list, open_list);
                new_pom->depth = MIN(new_pom->depth, depth+1);
                e_ptr->add_child(new_pom, next_act);
            }
        }

        if(e_ptr->action_set.size() > 0)
            edge_list.insert(make_pair(obs, e_ptr));
    }

    void PomdpState::expand(SetList& full_list, SetList& open_list, shared_ptr<MapUtils>& map_ut) {
        has_expand = true;
        if(convex_num == -1 || robot_num == -1) {
            convex_num = map_ut->m->cell_num;
            robot_num = map_ut->robot_num;
        }

        // loop through all possible observations 
        // (already observed)
        VectorXf::Index min_idx;
        FOR_RANGE(o, convex_num) {
            VectorXf target_belief = VectorXf::Zero(convex_num);
            target_belief(o) = 1.0;
            add_edge(target_belief, full_list, open_list, o, map_ut);
        }
        
        // not observed
        int obs = convex_num;
        VectorXf new_belief = target;
        // FOR_RANGE(i, robot_num)
            // new_belief(robot(i)) = 0;
        new_belief = new_belief.transpose() * map_ut->ctrans_prob;
        FOR_RANGE(i, robot_num)
            new_belief(robot(i)) = 0;
        if(new_belief.sum() == 0)
            return;
        format_belief(new_belief);
        add_edge(new_belief, full_list, open_list, obs, map_ut);
    }

    void PomdpState::update(shared_ptr<MapUtils>& mu) {
        VectorXd obs_prob = VectorXd::Zero(convex_num + 1);
        FOR_RANGE(i, robot_num) {
            obs_prob(robot(i)) = double(target(robot(i)));
        }
        double sum = obs_prob.sum();
        obs_prob(convex_num) = 1 - sum;

        sum = 0;
        for(auto kv : edge_list) {
            kv.second->update(mu, robot);
            sum += obs_prob(kv.first) * kv.second->max_v;
            // kv.second->print();
        }
        value = sum;
    }

    void TransState::add_edge(VectorXf& tmp_target, SetList& full_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu) {
        // all edge should lead to base
        shared_ptr<Edge> e_ptr = make_shared<Edge>(obs, T_TRANS);
        vector<IntPair> base_list = mu->m->convex_cover[int(robot(0))];
        for(auto base_2d : base_list) {
            int base_1d = mu->m->get1d_grid(base_2d);
            VectorXi act(1); act << base_1d;
            VectorXi robot_vec(1); robot_vec << base_1d;
            shared_ptr<BaseState> b_ptr = make_shared<BaseState>();
            b_ptr->robot = robot_vec;
            b_ptr->target = tmp_target;
            auto new_b_ptr = find_child(b_ptr, full_list, open_list);
            new_b_ptr->depth = MIN(new_b_ptr->depth, depth+1);
            e_ptr->add_child(new_b_ptr, act);
        }

        if(e_ptr->action_set.size() > 0)
            edge_list.insert(make_pair(obs, e_ptr));
    }

    void TransState::expand(SetList& full_list, SetList& open_list, shared_ptr<MapUtils>& map_ut) {
        has_expand = true;
        
        // expand all base location with 
        // current convex hull
        vector<IntPair> base_list = map_ut->m->convex_cover[int(target(0))];
        for(auto base_2d : base_list) {
            int base_1d = map_ut->m->get1d_grid(base_2d);
            VectorXf target_vec(1); target_vec << float(base_1d);
            add_edge(target_vec, full_list, open_list, base_1d, map_ut);
        }

    }

    void TransState::update(shared_ptr<MapUtils>& mu) {
        double sum = 0;
        for(auto kv : edge_list) {
            kv.second->update(mu, robot);
            sum += kv.second->max_v;
        }
        if(edge_list.size() > 0)
            value = sum / double(edge_list.size());
    }

    void BaseState::add_edge(VectorXf& tmp_target, SetList& full_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu) {
        shared_ptr<Edge> e_ptr = make_shared<Edge>(obs, T_BASE);

        // check whether expand pomdp or base
        if(obs == -1) {
            vector<int> r_convex;
            mu->get_convex_from_base1d(r_convex, robot(0));
            
            for(auto act : mu->action_mesh) {
                if(find(act.begin(), act.end(), robot(0)) == act.end())
                    continue;
                
                shared_ptr<PomdpState> pom_ptr = make_shared<PomdpState>();
                VectorXi next_r = Eigen::Map<VectorXi, Eigen::Unaligned>(act.data(), act.size());
                pom_ptr->robot = next_r;
                pom_ptr->target = tmp_target;
                auto new_pom_ptr = find_child(pom_ptr, full_list, open_list);
                VectorXi action(next_r); 
                new_pom_ptr->depth = MIN(new_pom_ptr->depth, depth+1);
                e_ptr->add_child(new_pom_ptr, action);
            }
        }
        
        // base state
        else {
            int r = robot(0);
            FOR_RANGE(a, ACT_LENGTH) {
                int next_r = mu->base_transition(r, a);
                shared_ptr<BaseState> b_ptr = make_shared<BaseState>();
                VectorXi next_r_vec(1); next_r_vec << next_r;
                b_ptr->robot = next_r_vec;
                b_ptr->target = tmp_target;
                auto new_b_ptr = find_child(b_ptr, full_list, open_list);
                VectorXi act(1); act << a;
                new_b_ptr->depth = MIN(new_b_ptr->depth, depth+1);
                e_ptr->add_child(new_b_ptr, act);
            }
        }

        // potential bug since key redundent
        if(e_ptr->action_set.size() > 0) 
            edge_list.insert(make_pair(obs, e_ptr));
    }

    void BaseState::expand(SetList& full_list, SetList& open_list, shared_ptr<MapUtils>& map_ut) {
        has_expand = true;
        
        // check caught
        if(float(robot(0)) == target(0)) {
            is_term = true;
            return;
        }

        // check whether go back to pomdp
        vector<int> t_convex;
        map_ut->get_convex_from_base1d(t_convex, int(target(0)));
        int conv_num = -1;
        if(!map_ut->is_visible(conv_num, robot(0), int(target(0)))) {
            // back to pomdp
            float uni_belief = 1 / float(t_convex.size());
            VectorXf t_belief = VectorXf::Zero(map_ut->m->cell_num);
            for(int tc : t_convex) 
                t_belief(tc) = uni_belief;
            
            add_edge(t_belief, full_list, open_list, -1, map_ut);
            return;
        }

        // stay in base
        // the target should be visible
        int t = int(target(0));
        FOR_RANGE(a, ACT_LENGTH) {
            int next_t = map_ut->base_transition(t, a);
            VectorXf tmp_t(1); tmp_t << float(next_t);
            add_edge(tmp_t, full_list, open_list, next_t, map_ut);
        }
        
    }

    void BaseState::update(shared_ptr<MapUtils>& mu) {
        double sum = 0;
        for(auto kv : edge_list) {
            kv.second->update(mu, robot);
            sum += kv.second->max_v;
        }
        if(edge_list.size() > 0)
            value = sum / double(edge_list.size());
    }

    void Edge::add_child(const shared_ptr<State>& s_ptr, VectorXi& act) {
        action_map.insert(make_pair(act, s_ptr));
        children.push_back(s_ptr);
        action_set.push_back(act);
    }

    void Edge::update(shared_ptr<MapUtils>& mu, const VectorXi& robot) {
        int asize = action_set.size();
        if(asize == 0)
            return;
        VectorXd q_vec(asize);
        double reward = 0;

        if(ptype == T_BASE) {
            FOR_RANGE(i, asize) {
                auto child = action_map[action_set[i]];
                if(child->stype == T_POMDP) {
                    q_vec(i) = child->value * DISCOUNT;
                }
                else if(child->stype == T_BASE) {
                    if(child->is_term) reward = mu->CATCH_REWARD;
                    else reward = mu->base_reward(robot(0), action_set[i](0));
                    q_vec(i) = reward + child->value * DISCOUNT / double(asize);
                } 
            }
        }
        else {
            FOR_RANGE(i, asize)
                q_vec(i) = action_map[action_set[i]]->value * DISCOUNT / double(asize);
        }

        Eigen::Index idx;
        q_vec.maxCoeff(&idx);
        max_v = q_vec(idx);
        optimal_act_idx = idx;
        // print();
    }
};
