//
// Created by ERANHER on 9.6.2021.
//

#ifndef PE_AGENTGR_HPP
#define PE_AGENTGR_HPP
//
// Created by eranhe on 5/25/21.
//

#include <utility>

#include "GoalRec/PathRec.hpp"
#include "States/State.hpp"

class PRecAgent {


    agentEnum attackerID = agentEnum::A;
    agentEnum my_id;
    int max_speed;
    string home;
    GoalRecognition GR;

public:
    PRecAgent(int maxSpeedAgent, agentEnum agentId, string home1, int seed);

    Point get_action(State<> *s) ;
    ~PRecAgent() =default;

    void reset_policy() ;
    void policy_data() const ;
    std::vector<double>* TransitionAction(const State<> *s) const ;

    void intial_args(const std::vector<std::vector<Point>> &pathz,vector<double> &&path_probabilties);

};


void PRecAgent::intial_args(const vector<std::vector<Point>> &pathz, vector<double> &&path_probabilties) {
    this->GR.load_agent_paths(pathz,std::move(path_probabilties));

}

PRecAgent::PRecAgent(int maxSpeedAgent, agentEnum agentId, string home, int seed
):max_speed(maxSpeedAgent),my_id(agentId),home(std::move(home)), GR(seed) {

}

Point PRecAgent::get_action(State<> *s){

    GR.set_my_location(s->get_position(this->my_id));
    auto action  = GR.do_action(s->get_position(this->attackerID),
                                s->get_speed_ref(this->my_id));
    return action;
}


void PRecAgent::reset_policy() {
    this->GR.reset_ptr();

}

void PRecAgent::policy_data() const {
    printf("PRecAgent::No Data is available");
}


std::vector<double> *PRecAgent::TransitionAction(const State<> *s)const {
    return new std::vector<double>();
}




#endif //PE_AGENTGR_HPP
