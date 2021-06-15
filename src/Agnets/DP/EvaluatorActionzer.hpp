//
// Created by eranhe on 6/13/21.
//

#ifndef PE_EVALUATORACTIONZER_HPP
#define PE_EVALUATORACTIONZER_HPP
#include "States/State.hpp"
#include "../../utils/game_util.hpp"
#include "../Policer.hpp"
#include "utils/Jumper.h"
class Evaluator{

    agentEnum e=agentEnum::A;
    agentEnum p=agentEnum::D;
    std::shared_ptr<MemoryRtdp> memory_rtdp{};
    Rewards R = Rewards::getRewards();
public:

    Evaluator()=delete;

    explicit Evaluator(std::shared_ptr<MemoryRtdp> ptr):memory_rtdp(std::move(ptr)){}

    Cell evalute_state(const State<> &s,double transition_probability);

    Cell eval_states(vector<tuple<StatePoint, int, double>> &arr, State<> &s,Policer* policer);

    tuple<double, bool> EvalState2(const State<> &s);

    void set_jumps(State<> &s);

    Cell plan_rec_helper(State<> &s);

};



Cell Evaluator::eval_states(vector<tuple<StatePoint, int, double>> &arr,State<> &s,Policer* policer) {
    Cell expected_sum_reward=0;
    auto time_next = s.state_time+s.jump;
    int steps=s.jump;
    for(const auto& itemL:arr)
    {
        auto state_point = std::get<0>(itemL);
        s.set_speed(this->e,state_point.speed);
        s.set_position(this->e,state_point.pos);
        s.state_time=time_next; // fix the time step
        assert(std::get<1>(itemL)==time_next);
        set_jumps(s); // set the jumps
        policer->search(s);
        expected_sum_reward+=evalute_state(s,std::get<2>(itemL));
    }
    return expected_sum_reward*std::pow(R.discountF,steps);
}
Cell Evaluator::evalute_state(const State<> &s, double transition_probability) {
    double res=0;
    auto [val,isEndState]= this->EvalState2(s);
    if(!isEndState)
        val+=this->memory_rtdp->get_max_val(s);
    res+=val*transition_probability;
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
        return {R.GoalReward,true};
        return {R.GoalReward*x,true};
    }
    if (s.is_collusion(p,e))
    {
        // cout<<"[R.CollReward]"<<endl;
        return {R.CollReward,true};
    }
    return {R.Step_reward,false};
}

void Evaluator::set_jumps(State<>& s)
{
    s.jump = Jumper::get_jumps(s.get_position_ref(this->e),s.get_position_ref(this->p));
}

Cell Evaluator::plan_rec_helper(State<> &s) {
    auto list_successors = GoalRecognition::get_all_successors(s.budgets.ptr,s.jump);
    double sum_all=0.0;
    double expected_sum_reward=0.0;
    int steps = s.jump;
    uint t = s.jump+s.state_time;
    for(auto& item:list_successors)
    {
        sum_all=std::accumulate(item->goal_likelihood.begin(), item->goal_likelihood.end(),
                                       sum_all);
    }

    for(NodeG* i: list_successors)
    {
        double sum_prob = std::accumulate(i->goal_likelihood.begin(),i->goal_likelihood.end(),0.0);
        s.budgets=i;
        s.set_position(this->e,i->pos);
        s.set_speed(this->e,Point(0));
        s.state_time=t;
        expected_sum_reward+=evalute_state(s,sum_prob/sum_all);
    }

    return expected_sum_reward*std::pow(R.discountF,steps);

}


#endif //PE_EVALUATORACTIONZER_HPP
