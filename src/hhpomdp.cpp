#include "hhpomdp.hpp"

using namespace hhpomdp;

HHPOMDP::HHPOMDP(shared_ptr<Map>& input_map, int ri, MatrixXi& _robot) : _map(input_map), robot_num(ri), robot(_robot) {
    map_util = shared_ptr<MapUtils>(new MapUtils(_map, robot_num));

    Eigen::VectorXi r = map_util->get_robot_conv(robot);
    cout << "r:" << r.transpose() << endl;

    Eigen::VectorXf t = Eigen::VectorXf::Ones(_map->cell_num);
    t /= t.sum();
    fvi = make_shared<ForwardValueIteration>(map_util, map_util->robot_num, r, t);
}


bool HHPOMDP::start() {
    std::cout << "starting... " << std::endl;
    const clock_t begin_time = clock();
    fvi->init();
    cout << "time: " << float( clock () - begin_time  ) /  CLOCKS_PER_SEC << " s."<< endl;
    return true;
}

