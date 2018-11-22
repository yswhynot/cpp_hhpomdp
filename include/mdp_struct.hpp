#ifndef MDP_STRUCT_H
#define MDP_STRUCT_H

#include "type_struct.hpp"
#include "map_utils.hpp"

using namespace std;
using namespace Eigen;

namespace hhpomdp {

    enum StateType {
        T_POMDP, T_TRANS, T_BASE, T_COUNT
    };

    struct State; 
    struct PomdpState;
    struct TransState;
    struct BaseState;
    struct Edge;

    typedef unordered_set< shared_ptr<State>, Deref::Hash, Deref::Compare> SetList;

    struct State {
        State() : has_expand(false), is_term(false), value(0), depth(numeric_limits<int>::max()) {}

        int id;
        StateType stype;
        map<int, shared_ptr<Edge> > edge_list;   // pomdp: int-obs, trans: int-(r, t)grid, base: int-target action
        string name;
        VectorXi robot;     // pomdp: convex id, trans: [0]convex id, base: [0] grid_1d
        VectorXf target;    // pomdp: belief vec, trans: [0] convex id, base: [0] grid_1d
        bool has_expand;
        double value;
        bool is_term;
        int depth;

        virtual void add_edge(VectorXf& tmp_target, SetList& close_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu) {}
        virtual void expand(SetList& close_list, SetList& open_list, shared_ptr<MapUtils>& map_ut) {}
        virtual void update(shared_ptr<MapUtils>& mu) {}
        void print() {
            cout << "id:" << id << ", stype:" << stype << ", robot:" << robot.transpose() << ", target:" << target.transpose() << ", value:" << value << endl;
        }

        bool operator==(State& in) const {
            if(stype != in.stype)
                return false;
            else
                return (target.isApprox(in.target) 
                    && robot.isApprox(in.robot));
        }
    };

    struct PomdpState : State {
        PomdpState() : State(), convex_num(-1), robot_num(-1) {stype = T_POMDP;}

        void set_belief(VectorXf& input);

        bool operator==(TransState& in) {return false;}
        bool operator==(BaseState& in) {return false;}

        void add_edge(VectorXf& tmp_target, SetList& close_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu); 
        void expand(SetList& close_list, SetList& open_list, shared_ptr<MapUtils>& map_ut);
        void update(shared_ptr<MapUtils>& mu);

        int convex_num;
        int robot_num;
    };

    struct TransState : State {
        TransState() : State() {stype = T_TRANS;}

        bool operator==(PomdpState& in) {return false;}
        bool operator==(BaseState& in) {return false;}

        void add_edge(VectorXf& tmp_target, SetList& close_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu);
        void expand(SetList& close_list, SetList& open_list, shared_ptr<MapUtils>& map_ut);
        void update(shared_ptr<MapUtils>& mu);
    };

    struct BaseState : State {
        BaseState() : State() {stype = T_BASE;}
        
        bool operator==(PomdpState& in) {return false;}
        bool operator==(TransState& in) {return false;}
        void add_edge(VectorXf& tmp_target, SetList& close_list, SetList& open_list, int obs, shared_ptr<MapUtils>& mu);
        void expand(SetList& close_list, SetList& open_list, shared_ptr<MapUtils>& map_ut);
        void update(shared_ptr<MapUtils>& mu);

    };

    struct Edge {
        Edge(int input_o, StateType input_t) : obs(input_o), max_v(0), ptype(input_t) {}

        void add_child(const shared_ptr<State>& s_ptr, VectorXi& act);
        void update(shared_ptr<MapUtils>& mu, const VectorXi& p_ptr);
        void print() {
            cout << "edge obs:" << obs << endl;
            FOR_RANGE(i, action_set.size()) {
                cout << "action_set(" << i << "):" << action_set[i].transpose() << ", ";
                cout << "act child:"; children[i]->print();
            }
            cout << "optimal_act:" << action_set[optimal_act_idx].transpose() << ", max_v:" << max_v << ", child:";
            action_map[action_set[optimal_act_idx]]->print();
        }

        int obs;        // key in edge_list
        StateType ptype;
        vector< shared_ptr<State> > children; // child states, index = corresponding action index
        vector<VectorXi> action_set;
        unordered_map<VectorXi, shared_ptr<State>, matrix_hash<VectorXi> > action_map;
        int optimal_act_idx;    // size = robot_num (pomdp), = 1 (base), invalid (trans)
        double max_v;
    };
};

namespace std {
    template<> struct hash<hhpomdp::State> {
        typedef hhpomdp::State argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const& s) const noexcept
        {
            result_type const h1 ( std::hash<int>()(int(s.stype)));
            result_type const h2 (hhpomdp::matrix_hash<VectorXi>()(s.robot) );
            result_type const h3 (hhpomdp::matrix_hash<VectorXf>()(s.target) );

            size_t seed = 0;
            boost::hash_combine(seed, h1);
            boost::hash_combine(seed, h2);
            boost::hash_combine(seed, h3);
            return seed;

        }

    };

};

#endif
