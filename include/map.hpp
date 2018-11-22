#ifndef MAP_H
#define MAP_H

#include <Eigen/Sparse>

#include "type_struct.hpp"

using namespace std;

namespace hhpomdp {

class Map {
public:
    Map(int input_id) : id(input_id) { init_map(); }

private:
    void generate_map_hash();
    void get_grid_list(IntPair& top_left, IntPair& bottom_right, vector<IntPair>& result);
    void init_map();
    void print_env();
    void print_vector_3d(vector_3d& vec_ptr);

public:
    // utils functions
    int get1d_grid(int x, int y);
    int get1d_grid(IntPair& ip);
    void get2d_grid(IntPair& result, int p);

public:
    int id;
    int width;
    int height;
    int cell_num;

    MatrixXb env_map;
    MatrixXb convex_trans;

    vector< vector<IntPair> > convex_cover;

    vector_3d convex_id_map; // [x][y] = vector of convex hull ids

};


}

#endif
