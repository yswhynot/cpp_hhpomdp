#include "forward_vi.hpp"

using namespace std;
using namespace Eigen;

namespace hhpomdp {
ForwardValueIteration::ForwardValueIteration(shared_ptr<MapUtils>& input_mu, int ri, VectorXi& robot_dis, VectorXf& target_belief) : robot_num(ri), _robot_dis(robot_dis), _target_belief(target_belief), _map_util(input_mu) {
	_map = _map_util->m;
    max_depth = 0;
    cout << "Map " << _map->id << ", #robot: " << robot_num << ", robot start:" << robot_dis.transpose() << ", target start:" << target_belief.transpose() << endl;
}

void ForwardValueIteration::init() {
	max_iteration = 100000;
    stop_thresh = 0.0001;

    root = make_shared<PomdpState>();
    root->robot = _robot_dis;
    root->set_belief(_target_belief);
    root->depth = 0;
    open_list.insert(root);
    full_list.insert(root);
   
    expand_tree();
    iterate(root);
}

void ForwardValueIteration::expand_tree() {
	// perform forward exploration of belief states
	cout << "Expanding policy tree..." << endl;
    while(open_list.size() > 0) {
        auto node_ptr = *(open_list.begin());
        max_depth = MAX(max_depth, node_ptr->depth);
        // node_ptr->print();
        StateType st = node_ptr->stype;
        open_list.erase(node_ptr);

        if(st == T_POMDP) {
            shared_ptr<PomdpState> cast_ptr = dynamic_pointer_cast<PomdpState>(node_ptr);
            cast_ptr->expand(full_list, open_list, _map_util);
        } else if(st == T_TRANS) {
            shared_ptr<TransState> cast_ptr = dynamic_pointer_cast<TransState>(node_ptr);
            cast_ptr->expand(full_list, open_list, _map_util);
        } else if(st == T_BASE) {
            shared_ptr<BaseState> cast_ptr = dynamic_pointer_cast<BaseState>(node_ptr);
            cast_ptr->expand(full_list, open_list, _map_util);
        }
    }
    cout << "Expanded " << full_list.size() << " nodes, max_depth: " << max_depth << endl;
}

void ForwardValueIteration::iterate(const shared_ptr<State>& root) {
    double prev_v = root->value;
    int it = 0;
    bool is_start = false;
    double root_thresh = pow(DISCOUNT, max_depth) * _map_util->CATCH_REWARD;
    stop_thresh = MAX(root_thresh, stop_thresh);
    cout << "Start iteration with stop_thresh: " << stop_thresh << endl;
    while(it < max_iteration) {
        for(auto s : full_list) {
            s->update(_map_util);
            // s->print();
        }

        if(is_start && abs(root->value - prev_v) < stop_thresh) break;

        if(root->value != 0)
            is_start = true;

        cout << "it: " << it << ", prev_v: " << prev_v << ", diff: " << abs(root->value - prev_v) << endl;
        prev_v = root->value;
        it ++;
    }
    cout << "Converged in " << it << " iterations." << endl;

}

void ForwardValueIteration::save_policy() {

}

};
