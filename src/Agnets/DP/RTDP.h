//
// Created by eranhe on 6/12/21.
//

#ifndef PE_RTDP_H
#define PE_RTDP_H

#include "States/State.hpp"
#include "RtdpUtil.h"
#include "MemoryRTDP.hpp"
#include "Attacker/StaticPolicy.hpp"
typedef RtdpUtils::StackActionzer<RtdpUtils::StackItemRtdp> StackObject;

class RTDP{

    agentEnum my_id = agentEnum::D;
    MemoryRtdp memo_rtdp;
    StackObject stack;


public:
    explicit RTDP(const StaticPolicy *evder);
    Point get_action(const State<> &s);
    void reset_policy();
    agentEnum get_id(){return my_id;}

};

RTDP::RTDP(const StaticPolicy *evder) {
    memo_rtdp = MemoryRtdp();
    stack=StackObject();

}

Point RTDP::get_action(const State<> &s) {

    //get the argmax action in Q table
    auto [action_i,entryID] = memo_rtdp.get_argMAx(s);

    stack.inset_to_stack(RtdpUtils::StackItemRtdp(State(s),std::move(action_i),entryID));

    return Point(0);
}

void RTDP::reset_policy() {
    stack.print_stak();
    stack.clear();
}



#endif //PE_RTDP_H
