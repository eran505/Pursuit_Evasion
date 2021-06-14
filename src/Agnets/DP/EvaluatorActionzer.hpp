//
// Created by eranhe on 6/13/21.
//

#ifndef PE_EVALUATORACTIONZER_HPP
#define PE_EVALUATORACTIONZER_HPP
#include "States/State.hpp"
#include "../../utils/game_util.hpp"
#include "../Policer.hpp"
class Evaluator{

    agentEnum e=agentEnum::A;
    agentEnum p=agentEnum::D;
    std::shared_ptr<MemoryRtdp> memory_rtdp;
    Rewards R = Rewards::getRewards();
public:

    Evaluator()=delete;

    explicit Evaluator(std::shared_ptr<MemoryRtdp> ptr):memory_rtdp(std::move(ptr)){}

    Cell evalute_state(const State<> &s,double transition_probability);

    Cell eval_states(vector<tuple<StatePoint, int, double>> &arr, State<> &s,DPGoalRec& policer);

    tuple<double, bool> EvalState2(const State<> &s);
};



Cell Evaluator::eval_states(vector<tuple<StatePoint, int, double>> &arr,State<> &s,DPGoalRec& policer) {
    Cell expected_sum_reward=0;
    for(const auto& itemL:arr)
    {
        auto state_point = std::get<0>(itemL);
        s.set_speed(this->e,state_point.speed);
        s.set_position(this->e,state_point.pos);
        policer.search(s);
        expected_sum_reward+=evalute_state(s,std::get<2>(itemL));
    }
    return expected_sum_reward;
}
Cell Evaluator::evalute_state(const State<> &s, double transition_probability) {
    auto steps = s.jump;
    double res=0;
    auto [val,isEndState]= this->EvalState2(s);
    if(!isEndState)
        val+=this->memory_rtdp->get_max_val(s);
    res+=val*transition_probability*std::pow(R.discountF,steps);
#ifdef PRINT
    cout<<"[evalute_state] "<<s.to_string_state()<<"  vla="<<res<<endl;
#endif
    return res;
}

tuple<double,bool> Evaluator::EvalState2(const State<> &s)
{
    if (s.g_grid->is_wall(s.get_position_ref(p)))
    {
        //cout<<"[R.WallReward]"<<endl;
        return {R.WallReward,true};
    }
    if (auto x = s.isGoal(e);x>=0)
    {
        // cout<<"[R.GoalReward]"<<endl;
        return {R.GoalReward*x,true};
    }
    if (s.is_collusion(p,e))
    {
        // cout<<"[R.CollReward]"<<endl;
        return {R.CollReward,true};
    }
    return {R.Step_reward,false};
}



#endif //PE_EVALUATORACTIONZER_HPP
