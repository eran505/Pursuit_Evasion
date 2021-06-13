//
// Created by ERANHER on 10.6.2021.
//

#ifndef PE_SIM_HPP
#define PE_SIM_HPP
#include "Attacker/StaticPolicy.hpp"
#include "GoalRec/AgentGR.hpp"
#include "utils/logeer.h"
template< typename P=PRecAgent, typename E=StaticPolicy >
class Emulator{
    E* evader;
    P* pursuer;
    State<> s_state;
    u_int32_t max_episodes=100;
    Logger logger;



public:

    Emulator(P* p , E* e , State<> &&s)
            :evader(e),pursuer(p),s_state(s){}

    void game_sim()
    {

        u_int32_t episodes=0;

        pursuer->reset_policy();
        evader->reset_policy();
        reset_state();
        //std::cout<<"run: "<<episodes<<"\tS:"<<s_state.to_string_state()<<endl;

        while(true)
        {


            set_jump();

            take_action_pursuer();
            take_action_evader();

            s_state.time_t+=s_state.jump;
            episodes++;
            //std::cout<<"run: "<<episodes<<"\tS:"<<s_state.to_string_state()<<endl;
            if (check_condition())
                break;



        }

        logger.print();
    }
    void main_loop(u_int32_t num_of_games)
    {
        auto s_start = State<>(s_state);
        for (int i = 0; i < num_of_games; ++i) {
            game_sim();
        }
    }
    bool check_condition()
    {

        const Point& pos_E = this->s_state.get_position(this->evader->get_id());
        const Point& pos_P = this->s_state.get_position(this->pursuer->get_id());
        //wall
        if(is_absolut_wall(pos_P))
        {
            #ifdef PRINT
            cout<<"[event] WallId => ";
            cout<<"[real] "<<s_state.to_string_state()<<endl;
            #endif
            logger.log_scalar_increment(Logger::WALL);
            return true;
        }
        //goal
        if(is_absolut_goal(pos_E))
        {
            #ifdef PRINT
            cout<<"[event] GoalId => ";
            cout<<"[real] "<<s_state.to_string_state()<<endl;
            #endif
            logger.log_scalar_increment(Logger::GOAL);
            return true;
        }
        //coll
        if(is_absolut_collision(pos_E,pos_P))
        {
            #ifdef PRINT
            cout<<"[event] CollId => ";
            cout<<"[real] "<<s_state.to_string_state()<<endl;
            #endif
            logger.log_scalar_increment(Logger::COLL);
            return true;
        }

        return false;
    }
    inline bool is_absolut_goal(const Point& pos_A)
    {
        // res == 0 - means that the goal is with zero reward for D
        // res>0 - means that the goal is -reward for D agent
        return s_state.g_grid->get_goal_reward(pos_A)>=0;
    }
    inline static bool is_absolut_collision(const Point& pos_D,const Point& pos_A)
    {
        return pos_A==pos_D;
    }
    inline bool is_absolut_wall(const Point& pos_D)
    {
        return this->s_state.g_grid->is_wall(pos_D);
    }

    void reset_state()
    {
        this->evader->set_start_point(s_state);
        this->pursuer->set_start_point(s_state);
    }
    void take_action_pursuer()
    {
        auto action_p = pursuer->get_action(s_state);
        s_state.applyAction(this->pursuer->get_id(),action_p,s_state.jump);
    }
    void take_action_evader()
    {
        evader->make_action(s_state);
    }
    void set_jump()
    {
        s_state.jump=1;
    }
};



#endif //PE_SIM_HPP
