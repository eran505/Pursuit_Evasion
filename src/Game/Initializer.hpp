//
// Created by eranhe on 6/6/21.
//

#ifndef PE_INITIALIZER_HPP
#define PE_INITIALIZER_HPP

#include "fileIO/processerCSV.hpp"
#include "Grid.hpp"
#include "States/State.hpp"
#include "Attacker/StaticPolicy.hpp"

class Initializer{

public:

    Initializer()=default;

    static std::unique_ptr<Grid> init_grid(const Point &grid_szie,const vector<Point> &gGoals,const vector<double> &probGoals)
    {
        return std::move(std::make_unique<Grid>(grid_szie,Point(0),gGoals,probGoals));
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
                                              conf.rRoutes,conf.home,lStartingPointGoal,listPointAttacker,conf._seed);


    }


};


#endif //PE_INITIALIZER_HPP
