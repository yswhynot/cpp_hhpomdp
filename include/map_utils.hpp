#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include "type_struct.hpp"
#include "map.hpp"

using namespace std;

namespace hhpomdp {

// implement min heap
struct MinHeapComp {
    bool operator()(const IntPair& a, const IntPair& b) {
        return a.second > b.second;
    }
};

struct MapUtils {
    const double OBS_REWARD = -1000;
    const double CATCH_REWARD = 1000;

    MapUtils(shared_ptr<Map>& input_map, int ri) : m(input_map), robot_num(ri) { init(); }
    void init();
    void generate_base_reward(Eigen::MatrixXd& result);
    void generate_base_transition(Eigen::MatrixXi& result);
    void generate_convex_action_set();
    void generate_target_transition(Eigen::MatrixXf& result);
    Eigen::VectorXi get_act_vec(int v, int cnum, int max_num);
    void get_action_remap(MatrixXi& result, MatrixXb& act_map);
    void get_convex_from_base1d(vector<int>& result, int base1d);
    float get_joint_reward(Eigen::VectorXi& robot_state, Eigen::VectorXi& robot_action, int target_state, int target_act);
    Eigen::VectorXi get_robot_conv(MatrixXi& r);
    bool get_next_grid(int& result, int current, int a);
    void get_next_pomdp_act(Eigen::VectorXi& result, Eigen::VectorXi& current, Eigen::VectorXi& next);
    bool is_visible(int& same_conv, int r_1d, int t_1d);
    void print_matrix();

    shared_ptr<Map> m;
    int robot_num;

    MatrixXi act_remap; // index: (cell_id, action which is also cell_id in the pomdp layer), value: next cell_id 

    Eigen::MatrixXd base_reward; // index: (grid_1d, action), value: reward
    Eigen::MatrixXi base_transition; // index: (grid_1d, action), value: next grid_1d, stay on the same grid if action invalid
    Eigen::MatrixXf ctrans_prob;

    vector<Eigen::VectorXi> convex_action_set;
    vector<Eigen::VectorXi> other_convex_set;
    vector< vector<int> > action_mesh;
};

};

#endif
