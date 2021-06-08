//
// Created by eranhe on 6/6/21.
//

#ifndef PE_POLICER_HPP
#define PE_POLICER_HPP

#include "utils/game_util.hpp"
#include "States/State.hpp"


typedef std::shared_ptr<unordered_map<string ,string>> dictionary;
using namespace AStar;
class Policy{

public:

    string home{};
    agentEnum id_agent{};
    unordered_map<int,Point*>* hashActionMap{};
    vector<Policy*> tran{};
    std::default_random_engine generator{};
    std::uniform_real_distribution<double> distribution{};

    Policy(int max_speed_agent,short agentID,string &_home,int seed=3)
            :max_speed(max_speed_agent),evalPolicy(false),home(_home),generator(seed){

        this->hashActionMap = Point::getDictAction();
        this->id_agent=static_cast<State::agentEnum>(agentID);
        distribution = std::uniform_real_distribution<double>(0.0,1.0);

    }
    [[nodiscard]] State::agentEnum get_id_name()const{ return id_agent;}
    [[nodiscard]] State::agentEnum GetId() const{ return id_agent;}
    double getRandom() {return distribution(generator);}


    void set_prefix_file_name(const string& prefix){prefix_file_name=prefix;}
    string get_prefix_file_name(const string& prefix) const{ return prefix_file_name;}

    virtual ~Policy(){
        //cout<<"~Policy"<<endl;
        for (auto &item : *this->hashActionMap)
            delete(item.second);
        delete(hashActionMap);
    }


    };

    void add_tran(Policy *ptr_tran)
    {
        this->tran.push_back(ptr_tran);
        if((tran).size()==1)
            cashID=tran[0]->id_agent;
    }
    void clear_tran(){
        this->tran.clear();
    }
    void applyActionToState(State *my_state, const Point &action )const{
        // change the budget according the budget function
        //this->budgetFunc(my_state, action);

        // append the prvoious speed to the new action
        my_state->applyAction(this->get_id_name(),action,this->max_speed);
    }
    void budgetFunc(State *state_now, const Point &action) const{


    }
    State* apply_action_state(State *my_state, const Point &action )const{

        my_state->applyAction(this->get_id_name(),action,this->max_speed);
        return my_state;
    }
    State& apply_action_state(State& my_state, const Point &action )const{

        my_state.applyAction(this->get_id_name(),action,this->max_speed);
        return my_state;
    }

    void set_id(State::agentEnum anEnum) {
        this->id_agent=anEnum;
    }
};

#endif //PE_POLICER_HPP
