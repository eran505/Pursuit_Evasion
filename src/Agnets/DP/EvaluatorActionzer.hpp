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
    int option_mode = -1;
    agentEnum e=agentEnum::A;
    agentEnum p=agentEnum::D;
    std::shared_ptr<MemoryRtdp> memory_rtdp{};
    Rewards R = Rewards::getRewards();
public:

    Evaluator()=delete;
    void set_option_mode(int x){option_mode=x;}
    explicit Evaluator(std::shared_ptr<MemoryRtdp> ptr):memory_rtdp(std::move(ptr)){}

    Cell evalute_state(const State<> &s,double transition_probability);

    Cell eval_states(vector<tuple<StatePoint, int, double>> &arr, State<> &s,Policer* policer);

    tuple<double, bool> EvalState2(const State<> &s);

    void set_jumps(State<> &s);

    //static bool is_one_plan(const State<> &state_);


    Cell plan_rec_helper(State<> &s);
    static  vector<vector<NodeG*>> NodeG_to_vectors(std::vector<NodeG*> l);
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
        policer->search_state(s);
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
    cout<<"[evalute_state] "<<s.to_string_state()<<"  V="<<res<<endl;
#endif
    return Cell(res);
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
    //assert(s.jump==1);
    auto list_successors = GoalRecognition::get_all_successors(s.budgets.ptr,s.jump);
    auto list_of_list_successors = NodeG_to_vectors(list_successors);
    double sum_all=0.0;
    double expected_sum_reward=0.0;
    double assert_num=0;
    int steps = s.jump;
    uint t = s.jump+s.state_time;
    for(auto& item:list_of_list_successors)
    {

        std::for_each(item.begin(),item.end(),[&sum_all](const NodeG *x){
            sum_all=std::accumulate(x->goal_likelihood.begin(), x->goal_likelihood.end(),
                                    sum_all);
        });

    }

    for(auto& i: list_of_list_successors)
    {
        double sum_prob = 0.0;
        std::for_each(i.begin(),i.end(),[&sum_prob](const NodeG *x){
            sum_prob=std::accumulate(x->goal_likelihood.begin(), x->goal_likelihood.end(),sum_prob);
        });
        s.budgets=i;
        s.set_position(this->e,i.front()->pos);
        s.set_speed(this->e,Point(0));
        s.state_time=t;
        if(option_mode==0)
            s.jump=1;
        else
            set_jumps(s);
        expected_sum_reward+=evalute_state(s,sum_prob/sum_all);
        assert_num+=sum_prob/sum_all;
    }

    assert(assert_num>0.99);
    return expected_sum_reward*std::pow(R.discountF,steps);

}

vector<vector<NodeG*>> Evaluator::NodeG_to_vectors(std::vector<NodeG*> l)
{
    vector<vector<NodeG*>> results;
    for(int i=0;i<l.size();i++)
    {
        std::vector<NodeG*> item;
        item.push_back(l[i]);
        for (int j = i+1; j < l.size(); ++j) {
            if(l[i]->pos==l[j]->pos) {
                item.push_back(l[j]);
            }
        }
        results.push_back(std::move(item));
    }
    return results;


}



#endif //PE_EVALUATORACTIONZER_HPP
