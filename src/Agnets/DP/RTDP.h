//
// Created by eranhe on 6/12/21.
//

#ifndef PE_RTDP_H
#define PE_RTDP_H

#include "States/State.hpp"
#include "RtdpUtil.h"
#include "MemoryRTDP.hpp"
#include "Attacker/StaticPolicy.hpp"
#include "ActionExpender.h"
#include "EvaluatorActionzer.hpp"
typedef RtdpUtils::StackActionzer<RtdpUtils::StackItemRtdp> StackObject;

class RTDP{

    agentEnum my_id = agentEnum::D;
    std::shared_ptr<MemoryRtdp> memo_rtdp;
    int max_speed = 1;
    StackObject stack;
    State<> start_point_state;
    //std::shared_ptr<NodeG> root_evader_tree;
    ActionExpend action_expend;
    Evaluator evaluator;

public:
    explicit RTDP(const StaticPolicy *evader,int seed);
    Point get_action(const State<> &s);
    void reset_policy();
    agentEnum get_id(){return my_id;}
    void set_start_point(State<> &s);
    void empty_stack();
    void update(const Point& a, State<>&& s,Entry id);
private:
    Cell bellman_update(State<> &&s , const Point &a);

    void do_SEQ(State<> &s, const Point &a);
};

RTDP::RTDP(const StaticPolicy *evader,int seed):
    memo_rtdp(std::make_shared<MemoryRtdp>(seed)),
    stack(),
    action_expend(evader),
    evaluator(memo_rtdp)
    {}


Point RTDP::get_action(const State<> &s) {

    //get the argmax action in Q table
    auto [action_i,entryID] = memo_rtdp->get_argMAx(s);

    stack.inset_to_stack(RtdpUtils::StackItemRtdp(State(s),std::move(action_i),entryID));

    return Point(0);
}

void RTDP::reset_policy() {
    stack.print_stak();
    empty_stack();
    stack.clear();
}



Cell RTDP::bellman_update(State<> &&s ,const  Point &a){

    //apply the action on the state
    cout<<s.to_string_state()<<endl;
    do_SEQ(s,a);
    cout<<s.to_string_state()<<endl;
    //send for expend + eval
    auto seq_states = this->action_expend.expander_attacker(s);
    Cell val = this->evaluator.eval_states(seq_states,s);
    return val;

}

void RTDP::set_start_point(State<> &s) {
    s.assignment(start_point_state,my_id);
}

void RTDP::empty_stack() {

    if(this->stack.is_empty()) return;

    while(!this->stack.is_empty()) {
        auto& item = this->stack.pop();
        //this->evaluator->change_scope_(&item.state);
        this->update(item.action,std::move(item.state),item.entryID);
    }
    this->stack.clear();
}

void RTDP::update(const Point &a, State<> &&s, Entry state_id) {

    Cell val = this->bellman_update(std::move(s),a);
    this->memo_rtdp->set_value_matrix(state_id,a.hashMeAction(Point::actionMax),val);
}

void RTDP::do_SEQ(State<> &s,const Point& a)
{
    auto a_tmp = Point(a);
    s.applyAction(this->my_id,a_tmp,this->max_speed,s.jump);
}


#endif //PE_RTDP_H
