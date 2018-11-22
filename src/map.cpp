#include "map.hpp"

using namespace std;

namespace hhpomdp {

void Map::init_map() {
    // This initialization is shitty but I cannot think of a very good way
    // very sad
    // super sad
    // QAQ

    const bool o = true;
    const bool z = false;

    if(id == 1) {
        Eigen::Matrix<bool, 4, 4> e;
        e << z, o, o, z,
              z, z, z, z,
              z, z, z, z,
              z, o, o, o;
        env_map = e;
        width = 4; height = 4;

        Eigen::Matrix<bool, 3, 3> c;
        c << o, o, z,
            o, o, o,
            z, o, o;
        convex_trans = c;
        cell_num = 3;

        for(int i = 0; i < cell_num; i++) {
            vector<IntPair> c_list;
            IntPair top_left; IntPair bottom_right;

            switch(i) {
                case 0: top_left = make_pair(0, 0); bottom_right = make_pair(3, 0);
                        break;
                case 1: top_left = make_pair(1, 0); bottom_right = make_pair(2, 3);
                        break;
                case 2: top_left = make_pair(0, 3); bottom_right = make_pair(2, 3);
                        break;
            }

            get_grid_list(top_left, bottom_right, c_list);
            convex_cover.push_back(c_list);
        }
    }

    else if(id == 3) {
        Eigen::Matrix<bool, 5, 5> e;
        e << z,z,z,z,z,
            z,z,z,z,z,
            z,o,z,o,z,
            z,o,z,o,z,
            z,z,z,z,z;
        env_map = e; width = 5; height = 5;

        Eigen::Matrix<bool, 5, 5> c;
        c << o,z,o,o,o,
            z,o,o,o,o,
            o,o,o,z,z,
            o,o,z,o,z,
            o,o,z,z,o;
        convex_trans = c; cell_num = 5;
        for(int i = 0; i < cell_num; i++) {
            vector<IntPair> c_list;
            IntPair top_left; IntPair bottom_right;

            switch(i) {
                case 0: top_left = make_pair(0, 0); bottom_right = make_pair(1, 4);
                        break;
                case 1: top_left = make_pair(4, 0); bottom_right = make_pair(4, 4);
                        break;
                case 2: top_left = make_pair(0, 0); bottom_right = make_pair(4, 0);
                        break;
                case 3: top_left = make_pair(0, 2); bottom_right = make_pair(4, 2);
                        break;
                case 4: top_left = make_pair(0, 4); bottom_right = make_pair(4, 4);
                        break;
            }

            get_grid_list(top_left, bottom_right, c_list);
            convex_cover.push_back(c_list);
        }
    }

    else if(id == 6) {
        Eigen::Matrix<bool, 5, 5> e;
        e << o,o,o,z,z,
            o,z,z,z,z,
            o,z,o,z,z,
            o,z,z,z,z,
            o,o,o,o,o;
        env_map = e; width = 5; height = 5;

        Eigen::Matrix<bool, 4, 4> c;
        c << o,o,z,o,
            o,o,o,z,
            z,o,o,o,
            o,z,o,o;
        convex_trans = c;
        cell_num = 4;

        for(int i = 0; i < cell_num; i++) {
            vector<IntPair> c_list;
            IntPair top_left; IntPair bottom_right;

            switch(i) {
                case 0: top_left = make_pair(1, 1); bottom_right = make_pair(3, 1);
                        break;
                case 1: top_left = make_pair(1, 1); bottom_right = make_pair(1, 4);
                        break;
                case 2: top_left = make_pair(0, 3); bottom_right = make_pair(3, 4);
                        break;
                case 3: top_left = make_pair(3, 1); bottom_right = make_pair(3, 4);
                        break;
            }

            get_grid_list(top_left, bottom_right, c_list);
            convex_cover.push_back(c_list);
        }
    }

    else if(id == 4) {
        Eigen::Matrix<bool, 12, 12> e;
        e << o,o,o,o,o,o,o,o,o,o,o,o,
            o,z,z,z,z,z,z,z,z,z,z,o,
            o,z,z,z,z,z,z,z,z,z,z,o,
            o,z,z,z,z,z,z,z,z,z,z,o,
            o,z,z,o,o,o,z,z,o,o,o,o,
            o,z,z,o,o,o,z,z,o,o,o,o,
            o,z,z,o,z,z,z,z,z,z,z,o,
            o,z,z,o,z,z,z,z,z,z,z,o,
            o,z,z,o,z,z,o,o,o,z,z,o,
            o,z,z,o,z,z,o,z,z,z,z,o,
            o,z,z,o,z,z,o,z,z,z,z,o,
            o,o,o,o,o,o,o,o,o,o,o,o;
        env_map = e; width = 12; height = 12;

        Eigen::Matrix<bool, 7, 7> c;
        c << o,o,z,z,z,z,z,
            o,o,o,z,z,z,z,
            z,o,o,o,z,z,z,
            z,z,o,o,o,o,z,
            z,z,z,o,o,z,z,
            z,z,z,o,z,o,o,
            z,z,z,z,z,o,o;
        convex_trans = c; cell_num = 7;
            
        for(int i = 0; i < cell_num; i++) {
            vector<IntPair> c_list;
            IntPair top_left; IntPair bottom_right;

            switch(i) {
                case 0: top_left = make_pair(1, 1); bottom_right = make_pair(10, 2);
                        break;
                case 1: top_left = make_pair(1, 1); bottom_right = make_pair(3, 10);
                        break;
                case 2: top_left = make_pair(1, 6); bottom_right = make_pair(7, 7);
                        break;
                case 3: top_left = make_pair(6, 4); bottom_right = make_pair(7, 10);
                        break;
                case 4: top_left = make_pair(6, 4); bottom_right = make_pair(10, 5);
                        break;
                case 5: top_left = make_pair(6, 9); bottom_right = make_pair(10, 10);
                        break;
                case 6: top_left = make_pair(9, 7); bottom_right = make_pair(10, 10);
                        break;
            }

            get_grid_list(top_left, bottom_right, c_list);
            convex_cover.push_back(c_list);
        }

    } else {
        cout << "Error: Wrong map id!" << endl;
    }

    generate_map_hash();

    cout << "Map " << id << " initialized!" << endl;
    // print_env();
}

void Map::generate_map_hash() {
    // init border_id_map and cell_id_map to the desired size
    vector_3d bmap, cmap;
    bmap.resize(height); 
    cmap.resize(height);
    for(int i = 0; i < height; i++) {
        bmap[i].resize(width);
        cmap[i].resize(width);
    }

    vector<IntPair>::iterator it;
    int x, y;

    for(int cid = 0; cid < convex_cover.size(); cid++) {
        vector<IntPair> c_array = convex_cover[cid];
        for(it = c_array.begin(); it != c_array.end(); it++) {
            x = it->first; y = it->second;
            cmap[x][y].push_back(cid);
        } // iterate through all cells in convex hull
    } // iterate through all convex hull cell ids
    convex_id_map = cmap;
}

void Map::get_grid_list(IntPair& top_left, IntPair& bottom_right, vector<IntPair>& result) {
    result.clear();
    int lx = top_left.first; int ly = top_left.second;
    int rx = bottom_right.first; int ry = bottom_right.second;

    for(int i = lx; i <= rx; i++) {
        for(int j = ly; j <= ry; j++) 
            result.push_back(make_pair(i, j));
    }
}


void Map::print_env() {
    cout << "printing environment map " << id << "...\n " << endl;

    cout << "env_map:\n" << env_map << endl << endl;
    cout << "convex_trans:\n" << convex_trans << endl << endl;

    vector<IntPair>::iterator it;
    int x, y;

    cout << "convex_cover:" << endl;
    for(int cid = 0; cid < convex_cover.size(); cid++) {
        vector<IntPair> c_array = convex_cover[cid];
        cout << "cid: " << cid << ", ";

        for(it = c_array.begin(); it != c_array.end(); it++) {
            x = it->first; y = it->second;
            printf("(%d, %d) ", x, y);
        } 
        cout << endl;
    } 

    cout << "convex_id_map:" << endl;
    print_vector_3d(convex_id_map);

}

void Map::print_vector_3d(vector_3d& vec) {
    for(int i = 0; i < 4; i++) {
        for(auto vec_x : vec) {
            for(auto vec_y : vec_x) {
                if(vec_y.size() > i) {
                    cout << vec_y[i];
                } else {
                    cout << "*";
                }
            }
            cout << endl;
        }
        cout << endl;
    }
    
}

int Map::get1d_grid(IntPair& ip) {
    return ip.first*width + ip.second;
}

int Map::get1d_grid(int x, int y) {
    return x*width + y;
}

void Map::get2d_grid(IntPair& result, int p) {
    int y = p % width;
    int x = int((p - y) / width);
    result = make_pair(x, y);
}

}

