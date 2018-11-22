#include <cstdlib>

#include "hhpomdp.hpp"
#include "map.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    cout << "generating policy for map " << argv[1] << endl;

    int robot_num = atoi(argv[2]);
    int map_id = atoi(argv[1]);
    MatrixXi robot; VectorXi target;
    if(map_id == 1) {
        robot = MatrixXi::Zero(robot_num, 2);
        VectorXi t(2); t<< 0, 3; target = t;
    }
    else if(map_id == 4) {
        robot = MatrixXi::Ones(robot_num, 2);
        VectorXi t(2); t<< 11, 11; target = t;
    }

    shared_ptr<hhpomdp::Map> m(new hhpomdp::Map(map_id));
    shared_ptr<hhpomdp::HHPOMDP> h(new hhpomdp::HHPOMDP(m, robot_num, robot));
    bool flag = h->start();
    return 0;
}
