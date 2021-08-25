//
// Created by eranhe on 6/6/21.
//

#ifndef PE_POLICER_HPP
#define PE_POLICER_HPP

#include "utils/game_util.hpp"
#include "States/State.hpp"
#include "GoalRec/PathRec.hpp"
#include "GoalRec/BTree.hpp"

class Policer{

public:
    agentEnum e = agentEnum::A;
    agentEnum p = agentEnum::D;

    virtual void update_state(State<> &s)
    {}

    virtual void reset_state(State<> &s)
    {}
    virtual void search_state(State<> &s)
    {}
    [[nodiscard]] virtual const BTree* get_object()const{return nullptr;};



};

typedef vector<vector<Point>> vector_paths;

class BePolicer:public Policer{
public:
    BTree GR;
    explicit BePolicer(int seed,int max_a,int max_d,const vector_paths& pathz, vector<double> &&prob_vec , const std::vector<u_int16_t> &names)
    :GR(seed,max_a,max_d)
    {
        GR.load_agent_paths(pathz,std::move(prob_vec),names);
    }
    void update_state(State<> &s) override
    {
        s.budgets.ptr = GR.get_next_step_by_loc(s.budgets.ptr,s.jump,s.get_position_ref(e));
    }

    void reset_state(State<> &s) override
    {
        s.budgets =  GR.get_start_ptr(s.get_position_ref(e));
    }
    void search_state(State<> &s) override
    {
        bool found=false;
        const Point& evader_position = s.get_position_ref(e);
        auto l = GR.get_all_successor(s.budgets.ptr,s.jump);
        for (auto& item:l){
            if(item->pos==evader_position){
                s.budgets=item; found=true;}
        }
        assert(found);
    }
    const BTree* get_object()const override {return &GR;}
};
//
//class DPGoalRec:public Policer{
//
//    GoalRecognition GR;
//
//public:
//
//
//    explicit DPGoalRec(int seed,int max_a,int max_d,const vector_paths& pathz, vector<double> &&prob_vec , const std::vector<u_int16_t> &names):GR(seed,max_a,max_d){
//        GR.load_agent_paths(pathz,std::move(prob_vec),names);
//    }
//
//    void update_state(State<> &s) override
//    {
////        if (s.to_string_state()=="32_A_(80, 153, 2, )_(1, 2, 0, )|D_(150, 132, 0, )_(0, 1, 0, )|_{ 24}_N_(52, 121, 0, )_16")
////            cout<<endl;
////        cout<<"s:\t"<<s.to_string_state()<<endl;
//        const Point& evader_position = s.get_position_ref(e);
//        GR.advance_curr_ptr(evader_position,s.jump);
//        auto l =  GR.get_list_cur();
////        cout<<"ptr:\t"<<l.front()->pos.to_str()<<endl;
//        assert(s.get_position_ref(e)==l.front()->pos);
//        s.budgets = l;
//
//    }
//    void reset_state(State<> &s) override
//    {
//        //const Point& evader_position = s.get_position_ref(e);
//        GR.reset_ptr();
//        //NodeG* ptr =  GR.get_curr_ptr();
//        s.budgets = GR.get_list_cur();
//    }
//    void search_state(State<> &s) override
//    {
//        const Point& evader_position = s.get_position_ref(e);
//        GR.search_tree_with_jumps(evader_position,s.jump,s.budgets.ptr);
//        s.budgets=GR.get_list_cur();
//        if (!(s.budgets.ptr.front()->pos==s.get_position_ref(agentEnum::A))){
//            cout<<s.to_string_state()<<endl;
//            assert(false);
//        }
//    }
//
//        // [s] = F(time_step,loc,path_id) -> {B states} , loc , time_step
//        // loc_time s
//};




#endif //PE_POLICER_HPP
