#ifndef SOLVER_CORE_H
#define SOLVER_CORE_H

#include "type_struct.hpp"
#include "mdp_struct.hpp"
#include "map_utils.hpp"

using namespace std;
using namespace Eigen;

namespace hhpomdp {

struct SolverCore {
    // ~SolverCore() {}
    virtual void init() {}
    virtual void save_policy() {}

    int robot_num;
    VectorXi _robot_dis;
    VectorXf _target_belief;
    shared_ptr<Map> _map;
    shared_ptr<MapUtils> _map_util;
    
};

};

#endif
