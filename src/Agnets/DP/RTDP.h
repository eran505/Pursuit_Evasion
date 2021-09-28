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
#include "../TrajectoriesTree.hpp"
#include "../Policer.hpp"
#include <type_traits>
#include <typeinfo>


typedef RtdpUtils::StackActionzer<RtdpUtils::StackItemRtdp> StackObject;

class RTDP{

    agentEnum my_id = agentEnum::D;
    std::shared_ptr<MemoryRtdp> memo_rtdp;
    int max_speed = 1;
    StackObject stack;
    State<> start_point_state;
    Point start_point;
    //std::shared_ptr<NodeG> root_evader_tree;
    ActionExpend action_expend;
    Evaluator evaluator;
    std::unique_ptr<Policer> policer;
    int mode =0;
public:
    explicit RTDP(const StaticPolicy *evader,const configGame& conf);

    RTDP(const StaticPolicy *evader,const configGame& conf,int idx);

    Point get_action(const State<> &s);
    void reset_policy();
    agentEnum get_id(){return my_id;}
    void set_start_point(State<> &s);
    void empty_stack();
    void update(const Point& a, State<>&& s,Entry id);
    [[nodiscard]] int get_max_speed() const{return max_speed;}
    void update_state(State<> &s);
    void set_Q_table(std::unique_ptr<Table>&& t){this->memo_rtdp->set_Q_table(std::move(t));}

    //std::vector<State<>> retrun_list_h(){return this->memo_rtdp->retrun_list_h();}

    std::unique_ptr<Table> get_Q_tabel(){ return this->memo_rtdp->get_Q_table();}

    size_t num_states_gen(){return this->memo_rtdp->get_size_state_map();}

    void update_B_tree(int path);
        std::unordered_map<u_int64_t,State<>> get_map_state(){ return std::move(this->memo_rtdp->get_map_state());}
private:
    Cell bellman_update(State<> &&s , const Point &a);
    void do_SEQ(State<> &s, const Point &a);


};

RTDP::RTDP(const StaticPolicy *evader,const configGame& conf):

    memo_rtdp(std::make_shared<MemoryRtdp>(conf._seed,evader->list_only_pos(),evader->get_copy_probabilities(),evader->get_paths_names(), conf.mode,conf.options,conf.h,conf.maxD)),
    stack(),
    action_expend(evader,conf.mode),
    evaluator(memo_rtdp),
    start_point(conf.posDefender.front()),
    policer(nullptr), mode(conf.mode)
    {
        if (this->mode==1)
            policer = std::make_unique<BePolicer>(conf._seed,conf.maxA,conf.maxD,evader->list_only_pos(),evader->get_copy_probabilities(),evader->get_paths_names());
        else policer = std::make_unique<Policer>();

        memo_rtdp->set_print_mode(conf.levelz);
        evaluator.set_option_mode(conf.options);
    }





Point RTDP::get_action(const State<> &s) {
    //get the argmax action in Q table
    auto [action_i,entryID] = memo_rtdp->get_argMAx(s);
    Point action = action_i;

    stack.inset_to_stack(RtdpUtils::StackItemRtdp(State(s),std::move(action_i),entryID));

    return action;
}

void RTDP::reset_policy() {
    //stack.print_stak();
    empty_stack();
    stack.clear();
}

void RTDP::update_state(State<> &s)
{
    policer->update_state(s);
    //cout<<s.to_string_state()<<endl;
}

void RTDP::update(const Point &a, State<> &&s, Entry state_id) {

    //cout<<"["<<s.to_string_state()<<","<<a.to_str()<<"]"<<endl;
    Cell val = this->bellman_update(std::move(s),a);

#ifdef PRINT
    cout<<state_id<<", "<<a.hashMeAction(Point::actionMax)<<"="<<val<<endl;
#endif
    this->memo_rtdp->set_value_matrix(state_id,a.hashMeAction(Point::actionMax),val);
}


Cell RTDP::bellman_update(State<> &&s ,const  Point &a){
#ifdef PRINT
    cout<<" [pop]  [S] "<<s.to_string_state()<<"  [A] "<<a.to_str()<<endl;
#endif
    do_SEQ(s,a);
#ifdef PRINT
    cout<<"[updated (P)] "<<s.to_string_state()<<endl;
#endif
    if(this->mode==1)
        return this->evaluator.plan_rec_helper(s,policer->get_object());

    auto seq_states = this->action_expend.expander_attacker(s);

    Cell val = this->evaluator.eval_states(seq_states,s,policer.get());

    return val;

}

void RTDP::set_start_point(State<> &s) {
    s.set_speed(my_id,Point(0));
    s.set_position(my_id,Point(start_point));
    policer->reset_state(s);

}

void RTDP::empty_stack() {

    if(this->stack.is_empty()) return;
#ifdef PRINT
    stack.print_stak();
#endif
    //this->stack.pop();
    while(!this->stack.is_empty()) {
        auto& item = this->stack.pop();
        //this->evaluator->change_scope_(&item.state);
        this->update(item.action,std::move(item.state),item.entryID);
    }
    this->stack.clear();
}


void RTDP::do_SEQ(State<> &s,const Point& a)
{
    auto a_tmp = Point(a);
    s.applyAction(this->my_id,a_tmp,this->max_speed,s.jump);
}

void RTDP::update_B_tree(int path) {
    BTree *ptrTree = this->policer->get_object();
    if (ptrTree== nullptr) return;
    ptrTree->path_evader=path;
}


#endif //PE_RTDP_H
