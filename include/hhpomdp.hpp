#ifndef HHPOMDP_H
#define HHPOMDP_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <memory>

#include "boost/thread.hpp"

#include "type_struct.hpp"
#include "map_utils.hpp"
#include "map.hpp"
#include "forward_vi.hpp"

namespace hhpomdp {
class HHPOMDP {
public:
    HHPOMDP(shared_ptr<Map>& input_map, int ri, MatrixXi& _robot);
    bool start();

    shared_ptr<MapUtils> map_util;
    shared_ptr<ForwardValueIteration> fvi;

private:
    shared_ptr<Map> _map;
    int robot_num;
    MatrixXi robot;
    VectorXi target;
};
}


#endif
