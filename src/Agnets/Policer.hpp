//
// Created by eranhe on 6/6/21.
//

#ifndef PE_POLICER_HPP
#define PE_POLICER_HPP

#include "utils/game_util.hpp"
#include "States/State.hpp"
#include "GoalRec/PathRec.hpp"
class Policer{

public:
    virtual void update_state(State<> &s)
    {}

    virtual void reset_state(State<> &s)
    {}
    virtual void search_state(State<> &s)
    {}



};
typedef vector<vector<Point>> vector_paths;
class DPGoalRec:public Policer{

    GoalRecognition GR;
    agentEnum e = agentEnum::A;
    agentEnum p = agentEnum::D;
public:

    explicit DPGoalRec(int seed,int max_a,int max_d,const vector_paths& pathz, vector<double> &&prob_vec ):GR(seed,max_a,max_d){
        GR.load_agent_paths(pathz,std::move(prob_vec));
    }

    void update_state(State<> &s)
    {
//        if (s.to_string_state()=="32_A_(80, 153, 2, )_(1, 2, 0, )|D_(150, 132, 0, )_(0, 1, 0, )|_{ 24}_N_(52, 121, 0, )_16")
//            cout<<endl;
//        cout<<"s:\t"<<s.to_string_state()<<endl;
        const Point& evader_position = s.get_position_ref(e);
        GR.advance_curr_ptr(evader_position,s.jump);
        auto l =  GR.get_list_cur();
//        cout<<"ptr:\t"<<l.front()->pos.to_str()<<endl;
        assert(s.get_position_ref(e)==l.front()->pos);
        s.budgets = l;

    }
    void reset_state(State<> &s)
    {
        //const Point& evader_position = s.get_position_ref(e);
        GR.reset_ptr();
        //NodeG* ptr =  GR.get_curr_ptr();
        s.budgets = GR.get_list_cur();
    }
    void search_state(State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        GR.search_tree_with_jumps(evader_position,s.jump,s.budgets.ptr);
        s.budgets=GR.get_list_cur();
        if (s.budgets.ptr.front()->pos!=s.get_position_ref(agentEnum::A)){
            cout<<s.to_string_state()<<endl;
            assert(false);
        }
    }


};




#endif //PE_POLICER_HPP
