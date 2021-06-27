//
// Created by eranhe on 6/27/21.
//

#ifndef PE_HEURISTICER_HPP
#define PE_HEURISTICER_HPP
#include "States/State.hpp"

typedef float Cell;
typedef u_int64_t Entry;

class Heuristicer{


    std::vector<Point> id_to_point=Point::getVectorActionUniqie();
    int max_speed_P;
    agentEnum Eid = agentEnum::A;
    agentEnum Pid = agentEnum::D;
    Rewards R = Rewards::getRewards();
    TrajectoriesTree trajectories_tree;
    std::function<int (const State<> &s)> H;

public:
    explicit Heuristicer(int maxP,int h,const vector<vector<Point>>& l_pathz):max_speed_P(maxP),trajectories_tree(l_pathz)
    {
        if (h==0) H = [](const State<> &ptrS) { return 0;};
        if (h==1) H = [this](const State<> &ptrS) { return this->trajectories_tree.get_min_steps_all_end(ptrS);};
        if (h==2) H = [this](const State<> &ptrS) { return this->trajectories_tree.get_min_steps_rel_end(ptrS);};
        if (h==3) H = [this](const State<> &ptrS) { return this->trajectories_tree.get_min_steps_next_all(ptrS);};
        if (h==4) H = [this](const State<> &ptrS) { return this->trajectories_tree.get_min_steps_next_foucs(ptrS);};

    }

    std::vector<Cell> heuristic(const State<>& s)const;




};

std::vector<Cell> Heuristicer::heuristic(const State<>& s)const
{

    vector<State<>> vec_q;
    auto oldState = State(s);
    auto row = std::vector<Cell>(27);
    size_t ctr=-1;
    for (const auto &item_action : this->id_to_point)
    {
        ctr++;
        // apply action state and let the envirmont to roll and check the reward/pos
        Point actionCur = item_action;

        double val;
        bool isWall = oldState.applyAction(Pid,actionCur,max_speed_P,oldState.jump);

        int step = H(oldState);


        if (isWall)
            val = (this->R.WallReward)*std::pow(R.discountF,oldState.jump);
        else
            val=(this->R.CollReward)*std::pow(R.discountF,step);

        oldState.assignment(s,this->Pid);
        // insert to Q table
        row[item_action.hashMeAction(Point::actionMax)]=val;
        //this->set_value_matrix(entry_index,item_action.hashMeAction(Point::actionMax) ,val);
    }
    return row;
}
#endif //PE_HEURISTICER_HPP
