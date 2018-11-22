#ifndef FORWARD_VI_H
#define FORWARD_VI_H

#include "type_struct.hpp"
#include "solver_core.hpp"
#include "map_utils.hpp"

using namespace std;
using namespace Eigen;

namespace hhpomdp {

struct ForwardValueIteration : SolverCore {
    ForwardValueIteration(shared_ptr<MapUtils>& input_mu, int ri, VectorXi& robot_dis, VectorXf& target_belief);
    void expand_tree();
    void init();
    void iterate(const shared_ptr<State>& root);
    void save_policy();

    int robot_num;
    VectorXi _robot_dis;
    VectorXf _target_belief;
    shared_ptr<Map> _map;
    shared_ptr<MapUtils> _map_util;
    shared_ptr<PomdpState> root;
    
    // shared_ptr<void> root;
    SetList open_list;
    SetList full_list;
	
    // iteration stopping criteria
    int max_iteration;
    float stop_thresh;
    int max_depth;
};

};

#endif
