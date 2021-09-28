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
template< typename P, typename E=StaticPolicy >
class Emulator{
    GoalRecognition GR;
    E* evader;
    P* pursuer;
    u_int16_t max_number_paths=-1;
    State<> s_state;
    Logger logger;
    int episodes=0;
    int done=0;
    int upper_time_bound = -1;
    std::function<int(const Point&,const Point&)> jump_func;
    std::vector<int> split_vec;

    // for debug
    State<> s_old;

public:

    Emulator(P* p , E* e , State<> &&s,const configGame &conf)
    :evader(e),pursuer(p),s_state(s),logger(conf),GR(conf._seed,conf.maxA,conf.maxD) {

        if (conf.options == 0){
            jump_func = [](const Point &, const Point &) { return 1; };
            cout<<"[options] disable"<<endl;
        }
        else if(conf.options==1) {
            jump_func = Jumper::get_jumps;
            cout<<"[options] enable"<<endl;
        } else assert(false);
        max_number_paths = this->evader->get_all_paths_size();
        GR.load_agent_paths(e->list_only_pos(),e->get_copy_probabilities(),e->get_paths_names());
        split_vec = GR.get_split_vector();
    }

    void game_sim()
    {

        episodes=0;
        // first clear P and only then clear E
        pursuer->reset_policy();
        evader->reset_policy();
        reset_state();
        while(true)
        {
            set_jump(); // check if the map state is correct
#ifdef PRINT
            std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
#endif
            //s_old=State<>(s_state);
            take_action_pursuer();
            take_action_evader();

            //cout<<s_state.to_string_state()<<endl;

            s_state.state_time+=s_state.jump;

            episodes++;


            //assert_state_old();

            if (check_condition())
                break;

        }

    }

//    void assert_state_old()
//    {
//        auto old_l_p = s_old.get_position_ref(this->pursuer->get_id());
//        auto l_p = s_state.get_position_ref(this->pursuer->get_id());
//        auto v_p = s_state.get_speed_ref(this->pursuer->get_id());
//        auto res = old_l_p+(v_p)*s_old.jump;
//        if(!(res==l_p))
//        {
//
//            cout<<"bugbug"<<"calc:"<<l_p.to_str()<<" : "<<res.to_str()<<endl;
//        }
//
//    }

    void main_loop(u_int32_t num_of_games)
    {
        upper_time_bound = check_upper_bound();
        auto s_start = State<>(s_state);
        for (int i = 0; i < num_of_games; ++i) {
            game_sim();
            if (!logger.eval())
                continue;
            //log the generated state number
            logger.log_scalar(pursuer->num_states_gen());
            logger.is_done();
            if (this->logger.get_done()) {
                done++;
                if (done == 1) {
                    break;
                }
            }
            else { done=0; }

        }
    }
    bool check_condition()
    {
        if(upper_time_bound>0)
            if (this->s_state.state_time>=upper_time_bound) {
                logger.log_scalar_increment(Logger::STOPED);
                return true;

            }
        const Point& pos_E = this->s_state.get_position(this->evader->get_id());
        const Point& pos_P = this->s_state.get_position(this->pursuer->get_id());
        //wall

        if(is_absolut_wall(pos_P))
        {
            #ifdef PRINT
            std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
            cout<<" ==> [event] WallId  "<<endl;
            #endif
            logger.log_scalar_increment(Logger::WALL);
            return true;
        }
        //goal
        if(is_absolut_goal(pos_E))
        {
            #ifdef PRINT
            std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
            cout<<" ==> [event] Goal  "<<endl;
            #endif
            logger.log_scalar_increment(Logger::GOAL);
            return true;
        }
        //coll
        if(is_absolut_collision(pos_E,pos_P))
        {
            #ifdef PRINT
            std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
            cout<<" ==> [event] Collision  "<<endl;
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
        pursuer->update_state(s_state);
        pursuer->update_B_tree(evader->get_currnt_path());

    }
    void take_action_pursuer()
    {
        //auto speed = s_state.get_speed(evader->get_id());
        //s_state.set_speed(evader->get_id(),Point(0));
        auto action_p = pursuer->get_action(s_state);
#ifdef PRINT
        //std::cout<<"run: "<<episodes<<"\t[state] "<<s_state.to_string_state()<<"\t";
        cout<<"[action] "<<action_p.to_str()<<endl;
#endif
        s_state.applyAction(this->pursuer->get_id(),action_p,this->pursuer->get_max_speed(),s_state.jump);

        //s_state.set_speed(evader->get_id(),speed);

    }
    void take_action_evader()
    {
        evader->make_action(s_state);

        pursuer->update_state(s_state);


    }
    void set_jump()
    {
        s_state.jump = jump_func(s_state.get_position_ref(evader->get_id()),s_state.get_position_ref(pursuer->get_id()));
        //s_state.jump = Jumper::get_jumps_splits(s_state.get_position_ref(evader->get_id()),s_state.get_position_ref(pursuer->get_id()),s_state.state_time,this->split_vec);

        //s_state.jump = Jumper::get_jumps(s_state.get_position_ref(evader->get_id()),s_state.get_position_ref(pursuer->get_id()));
        //cout<<s_state.to_string_state()<<"\t"<<"j:"<<s_state.jump<<"  d:"<<x<<endl;
        assert(s_state.jump>0);
    }
    int check_upper_bound()
    {
        std::vector<std::vector<Point>> all_pos_paths = this->evader->list_only_pos();
        if (all_pos_paths.size()==1)
            return all_pos_paths.front().size();
        else return -1;
    }

};


#endif //PE_SIM_HPP
