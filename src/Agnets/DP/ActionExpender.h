//
// Created by eranhe on 6/12/21.
//

#ifndef PE_ACTIONEXPENDER_H
#define PE_ACTIONEXPENDER_H
#include "States/State.hpp"
#include "Attacker/StaticPolicy.hpp"



typedef float valueType;
struct Rewards {
public:
    valueType CollReward = 1;
    valueType GoalReward = -0.9;
    valueType WallReward = -1;
    valueType Step_reward = 0;
    valueType discountF=0.987;//0.987;
    static Rewards getRewards()
    {
        return Rewards();
    }
};


class ActionExpend{
    StaticPolicy other;
    vector<tuple<StatePoint,int,double>> expander_attacker(const State<> &s_state,int jumps)
};


vector<tuple<StatePoint,int,double>> ActionExpend::expander_attacker(const State<> &s_state,int jumps)
{
    return other->weighted_next_partial_state(s_state,jumps);
}

#endif //PE_ACTIONEXPENDER_H
