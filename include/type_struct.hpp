#ifndef TYPE_STRUCT_H
#define TYPE_STRUCT_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/functional/hash.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

// rounding precision for belief space vector
#define ROUND_PRE 10
#define DISCOUNT 0.95

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

#define FOR_RANGE(i, n) for(int i = 0; i < n; i++)

namespace hhpomdp {

    typedef vector< vector < vector<int> > > vector_3d;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
    typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;
    typedef pair<int, int> IntPair;
    // typedef pair<Eigen::VectorXi, float> QPair;

    // hash for Eigen matrix and vectors for unordered_map
    template<typename T>
        struct matrix_hash : std::unary_function<T, size_t> {
            std::size_t operator()(T const& matrix) const {
                // Note that it is oblivious to the storage order of Eigen matrix (column- or
                // row-major). It will give you the same hash value for two different matrices if they
                // are the transpose of each other in different storage order.
                size_t seed = 0;
                for (size_t i = 0; i < matrix.size(); ++i) {
                    auto elem = *(matrix.data() + i);
                    seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
        };

    struct Deref {
        struct Hash {
        template <typename T>
            std::size_t operator() (std::shared_ptr<T> const &p) const {
                return std::hash<T>()(*p);
            }

        };
        struct Compare {
            template <typename T>
            size_t operator() (std::shared_ptr<T> const &a,
                    std::shared_ptr<T> const &b) const {
                return *a == *b;
            }
        };
    };


    enum BASE_ACTION : int {
        NORTH, SOUTH, WEST, EAST, NO_ACT, ACT_LENGTH
    };

    enum TRANS_ACTION : int {
        GO_TO_TRANS = -1,
        GO_TO_BASE = -2,
        GO_TO_POMDP = -3
    };

    enum STRATEGY : int {
        ST_UNIFORM, ST_GAME, ST_LENGTH
    };

    /**
     * Setting parameters
     **/
    const int strategy = ST_UNIFORM;
};

#endif
