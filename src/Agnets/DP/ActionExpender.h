//
// Created by eranhe on 6/12/21.
//

#ifndef PE_ACTIONEXPENDER_H
#define PE_ACTIONEXPENDER_H
#include "States/State.hpp"
#include "Attacker/StaticPolicy.hpp"






class ActionExpend{
    const StaticPolicy *other;
public:
    vector<tuple<StatePoint,int,double>> expander_attacker(const State<> &s_state);
    explicit ActionExpend(const StaticPolicy* _other):other(_other){}
};


vector<tuple<StatePoint,int,double>> ActionExpend::expander_attacker(const State<> &s_state)
{
    return other->weighted_next_partial_state(s_state,s_state.jump);
}

#endif //PE_ACTIONEXPENDER_H
