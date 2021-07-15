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
    std::vector<Point> start_p;
public:
    agentEnum get_id(){return my_id;}
    PRecAgent(int A_max,int D_max, agentEnum agentId, string home1,int seed,const std::vector<Point>& l_start);

    Point get_action(State<> &s) ;
    ~PRecAgent() =default;
    [[nodiscard]] int get_max_speed() const{return max_speed;}
    void reset_policy() ;
    void policy_data() const ;
    std::vector<double>* TransitionAction(const State<> *s) const ;
    void set_start_point(State<> &s){
        s.set_position(my_id,get_inital_place());
        s.set_speed(my_id,Point(0));
    };
    void update_state(State<> &s){};
    void intial_args(const std::vector<std::vector<Point>> &pathz,vector<double> &&path_probabilties,std::vector<u_int16_t> &&names);
    Point get_inital_place();
};


void PRecAgent::intial_args(const vector<std::vector<Point>> &pathz, vector<double> &&path_probabilties,std::vector<u_int16_t> &&names) {
    this->GR.load_agent_paths(pathz,std::move(path_probabilties),names);
}

PRecAgent::PRecAgent(int A_max,int D_max, agentEnum agentId, string home, int seed,const std::vector<Point>& l_start
):max_speed(D_max),my_id(agentId),home(std::move(home)), GR(seed,A_max,D_max) {
    start_p=l_start;
}

Point PRecAgent::get_action(State<> &s){

    GR.set_my_location(s.get_position(this->my_id));
    auto action  = GR.do_action(s.get_position(this->attackerID),
                                s.get_speed_ref(this->my_id));
    return action;
}


void PRecAgent::reset_policy() {
    GR.set_my_location(get_inital_place());
    this->GR.reset_ptr();

}

void PRecAgent::policy_data() const {
    printf("PRecAgent::No Data is available");
}


std::vector<double> *PRecAgent::TransitionAction(const State<> *s)const {
    return new std::vector<double>();
}

Point PRecAgent::get_inital_place() {
    return start_p.front();
}


#endif //PE_AGENTGR_HPP
