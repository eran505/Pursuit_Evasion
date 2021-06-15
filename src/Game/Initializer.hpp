//
// Created by eranhe on 6/6/21.
//

#ifndef PE_INITIALIZER_HPP
#define PE_INITIALIZER_HPP
#include "GoalRec/AgentGR.hpp"
#include "fileIO/processerCSV.hpp"
#include "Grid.hpp"
#include "States/State.hpp"
#include "Attacker/StaticPolicy.hpp"
#include "Agnets/DP/RTDP.h"
class Initializer{

public:

    Initializer()=default;

    static std::unique_ptr<Grid> init_grid(const Point &grid_szie,const vector<Point> &gGoals,const vector<double> &probGoals)
    {
        return std::move(std::make_unique<Grid>(grid_szie,Point(0),gGoals,probGoals));
    }

    static std::unique_ptr<PRecAgent> init_GR(configGame& conf,const StaticPolicy* e)
    {
        conf.eval_mode=-1;
        auto agent_p = std::make_unique<PRecAgent>(conf.maxD,agentEnum::D,conf.home,conf._seed,conf.posDefender);
        agent_p->intial_args(e->list_only_pos(),e->get_copy_probabilities());
        return std::move(agent_p);

    }
    static std::unique_ptr<RTDP> init_RTDP(const configGame& conf,const StaticPolicy* e)
    {
        Point start_p = conf.posDefender.front();
        auto agent_p = std::make_unique<RTDP>(e,conf._seed,conf.posDefender,conf.eval_mode);
        return std::move(agent_p);

    }
    static std::unique_ptr<StaticPolicy> init_attacker(const configGame& conf)
    {

        auto lStartingPointGoal = std::vector<std::pair<vector<Point>,double>>();
        auto gloz_l = conf.gGoals;
        assert(gloz_l.size()==conf.probGoals.size());
        for(int i=0;i<conf.probGoals.size();++i)
        {
            auto& ref_pos = lStartingPointGoal.emplace_back();
            ref_pos.first.emplace_back(gloz_l[i]);
            if(!conf.midPos.empty())
                ref_pos.first.insert(ref_pos.first.begin(),conf.midPos[i]);
            ref_pos.second=conf.probGoals[i];
        }
        std::vector<weightedPosition> listPointAttacker;
        for(auto &p:conf.posAttacker) listPointAttacker.emplace_back(Point(0,0,0),p,1.0);

        return std::make_unique<StaticPolicy>(conf.sizeGrid,conf.maxA,agentEnum::A,
                                              conf.rRoutes,conf.home,lStartingPointGoal,listPointAttacker,conf._seed,conf.idNumber);


    }


};


#endif //PE_INITIALIZER_HPP
