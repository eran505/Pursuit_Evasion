//
// Created by eranhe on 6/6/21.
//

#ifndef PE_POLICER_HPP
#define PE_POLICER_HPP

#include "utils/game_util.hpp"
#include "States/State.hpp"
#include "GoalRec/PathRec.hpp"

typedef vector<vector<Point>> vector_paths;
class DPGoalRec{

    GoalRecognition GR;
    agentEnum e = agentEnum::A;
    agentEnum p = agentEnum::D;
public:

    explicit DPGoalRec(int seed,const vector_paths& pathz, vector<double> &&prob_vec ):GR(seed){
        GR.load_agent_paths(pathz,std::move(prob_vec));
    }

    void update_state(State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        GR.advance_curr_ptr(evader_position);
        NodeG* ptr =  GR.get_curr_ptr();
        s.budgets = ptr;
    }
    void reset_state(State<> &s)
    {
        //const Point& evader_position = s.get_position_ref(e);
        GR.reset_ptr();
        NodeG* ptr =  GR.get_curr_ptr();
        s.budgets = ptr;
    }
    void search(State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        GR.search_tree(evader_position,s.budgets.ptr);
        s.budgets=GR.get_curr_ptr();
    }

};


class Dummy{

public:

    void update_state(State<> &s)
    {}
    void reset_state(State<> &s)
    {}

};

#endif //PE_POLICER_HPP
