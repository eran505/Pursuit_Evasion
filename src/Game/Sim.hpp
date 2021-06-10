//
// Created by ERANHER on 10.6.2021.
//

#ifndef PE_SIM_HPP
#define PE_SIM_HPP
#include "Attacker/StaticPolicy.hpp"
#include "GoalRec/AgentGR.hpp"

template< typename P=PRecAgent, typename E=StaticPolicy >
class Emulator{
    E* evader;
    P* pursuer;
    State<> s_state;
    u_int32_t max_episodes=10;
public:

    Emulator(P* p , E* e , State<> &&s)
            :evader(e),pursuer(p),s_state(s){}

    void main_loop()
    {
        u_int32_t episodes=0;
        while(episodes < this->max_episodes)
        {
            std::cout<<"run: "<<episodes<<endl;
            cout<<s_state.to_string_state()<<endl;
            auto action_p = pursuer->get_action(&s_state);
            auto action_p = evader->get_action(&s_state);
            episodes++;

        }
    }

};



#endif //PE_SIM_HPP
