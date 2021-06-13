//
// Created by eranhe on 6/13/21.
//

#ifndef PE_TRAJECTORIESTREE_HPP
#define PE_TRAJECTORIESTREE_HPP

#include "GoalRec/PathRec.hpp"

class TrajectoriesTree{

    GoalRecognition GR;

    explicit TrajectoriesTree(int seed,const std::vector<std::vector<Point>> &pathz,
        vector<double> &&path_probabilties):GR(seed){

        GR.load_agent_paths(pathz,path_probabilties);

    }



};

#endif //PE_TRAJECTORIESTREE_HPP
