//
// Created by ERANHER on 10.6.2021.
//

#ifndef PE_SIM_HPP
#define PE_SIM_HPP
#include "Attacker/StaticPolicy.hpp"
#include "GoalRec/AgentGR.hpp"
#include "utils/logeer.h"
#include "fileIO/processerCSV.hpp"
#include "utils/Jumper.h"
template< typename P=PRecAgent, typename E=StaticPolicy >
class Emulator{
    E* evader;
    P* pursuer;
    State<> s_state;
    Logger logger;
    int episodes=0;
    std::function<int(const Point&,const Point&)> jump_func;
public:

    Emulator(P* p , E* e , State<> &&s,const configGame &conf)
    :evader(e),pursuer(p),s_state(s),logger(conf)
    {
        jump_func=Jumper::get_jumps;
        if(conf.eval_mode==-1)
            jump_func = [](const Point&,const Point&){return 1;};
    }

    void game_sim()
    {

        episodes=0;

        pursuer->reset_policy();
        evader->reset_policy();
        reset_state();
        while(true)
        {
            set_jump();

            take_action_pursuer();

            take_action_evader();

            s_state.state_time+=s_state.jump;

            episodes++;

            if (check_condition())
                break;

        }

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
        s_state.state_time=0;
        s_state.jump=0;
    }
    void take_action_pursuer()
    {

        pursuer->update_state(s_state);
        auto action_p = pursuer->get_action(s_state);
#ifdef PRINT
        std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
        cout<<"[action] "<<action_p.to_str()<<endl;
#endif
        s_state.applyAction(this->pursuer->get_id(),action_p,this->pursuer->get_max_speed(),s_state.jump);

    }
    void take_action_evader()
    {
        evader->make_action(s_state);
    }
    void set_jump()
    {
        auto jumping = jump_func(s_state.get_position_ref(evader->get_id()),s_state.get_position_ref(pursuer->get_id()));
        //s_state.jump=1;
        s_state.jump=jumping;
    }


};


#endif //PE_SIM_HPP
