//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATICPOLICY_HPP
#define PE_STATICPOLICY_HPP


#include <utility>

#include "States/State.hpp"
#include "Attacker/PathMapper.hpp"
#include "Search/Astar.hpp"
#include "PathGenrator.hpp"
#include "utils/Rand.hpp"
typedef std::vector<StatePoint> APath;
typedef std::vector<pair<std::vector<Point>,double>> seq_goals;
typedef std::vector<weightedPosition> seq_starting;
shared_ptr<RandomSingleton> RandomSingleton::s_pSingleton;

class StaticPolicy{

    PathGenartor gen;
    std::unique_ptr<PathMapper<u_int32_t>>  mapper= nullptr;
    string home;
    agentEnum my_id;
    Randomizer randomizer;
    int max_speed;
    string exp_id;
    bool save_data = true;
public:
    StaticPolicy(const Point &gridSize,uint maxSpeed,agentEnum id,u_int16_t num_of_paths,std::string home_path,const seq_goals& goals_points,const seq_starting& starting_points,int seed,string id_exp,bool save=true,std::vector<int> names={})
    :gen(seed,gridSize,maxSpeed),randomizer(seed),max_speed(maxSpeed),exp_id(std::move(id_exp))
    {
        home = std::move(home_path);
        my_id=id;
        auto [list_pathz,probablity_list] = gen.geneate_path_loopV2(goals_points,starting_points,num_of_paths);
        make_mapper(list_pathz,probablity_list);
        save_data = save;

    }
    u_int32_t get_all_paths_size(){return mapper->get_all_paths_size();}
    void constant_path(int i)
    {
        mapper->set_constant(i);
    }
    void set_mapper(std::unique_ptr<PathMapper<u_int32_t>> mapper_ptr)
    {
        this->mapper=std::move(mapper_ptr);
    }

    std::unique_ptr<PathMapper<u_int32_t>> get_mapper()
    {
        return std::move(mapper);
    }
    vector<u_int16_t > get_paths_names()const{ return this->mapper->get_names_pathz();}

    agentEnum get_id(){return my_id;}
    u_int32_t get_max_len_path(){return mapper->get_max_path_size();}

    ~StaticPolicy(){
        policy_data();
    };
    int get_max_speed() const{return max_speed;}
    void set_start_point(State<> &s)
    {
        auto state_a = mapper->get_cur_position();
        s.set_position(my_id,std::move(state_a.pos));
        s.set_speed(my_id,std::move(state_a.speed));
    }
    void reset_policy()
    {
        mapper->random_choose_path(randomizer.get_double());
        //time_step=0;
    }
    void reset_policy_by_index_path(int index_p )
    {
        mapper->random_choose_path(randomizer.get_double());

    }
    void policy_data()const {

        if(!save_data) return;
        string pathFile=this->home+"/car_model/debug/p_"+exp_id+".csv";
        pathFile=this->home+"/car_model/debug/p.csv";
        //print Q table--------------------------------
        try{

            csvfile csv(pathFile,";"); // throws exceptions!
            auto p_list = mapper->get_all_probabilites_ref();
            auto pathz_list = mapper->get_all_pathz_ref();
            for (int i = 0; i < p_list.size(); ++i) {
                csv<<"P:"+std::to_string(p_list[i]);
                for(const auto &item:pathz_list[i])
                    csv<<item;
                csv<<endrow;
            }
        }
        catch (const std::exception &ex){std::cout << "Exception was thrown: " << ex.what() << std::endl;}

    }
    void make_action(State<> &s)
    {
        auto StatePoint = this->mapper->get_next_actual_state(s.jump);
        s.set_speed(this->get_id(),std::move(StatePoint.speed));
        s.set_position(this->get_id(),std::move(StatePoint.pos));

    }
    vector<tuple<StatePoint,int,double>> weighted_next_partial_state(const State<> &s,uint jumps) const{
        //cout<<"s: "<<s.to_string_state()<<"]  ";
        return mapper->get_next_states(get_hash_state(s),jumps);
    }
    std::vector<std::vector<StatePoint>> get_copy_pathz()const
    {
        return mapper->get_all_pathz();
    }
    std::vector<double> get_copy_probabilities()const
    {
        return mapper->get_all_probabilites();
    }
    std::vector<std::vector<Point>> list_only_pos()const
    {
        return mapper->get_all_pos();
    }

    void sanity_check()
    {
        vector<int> counter(mapper->get_all_probabilites().size());
        size_t max_iter= 10000;
        for (int i = 0; i < max_iter ; ++i) {
            auto ch = mapper->sanity_check(this->randomizer.get_double());
            counter[ch]+=1;
        }
        for (int i = 0; i < counter.size(); ++i) {
            cout<<"["<<i<<"] = "<<counter[i]/float(max_iter)<<endl;
        }
    }
    void get_action(State<> &s,int jumps=1)
    {
        auto state_point = this->mapper->get_next_actual_state(jumps);
        s.set_position(this->get_id(),std::move(state_point.pos));

        //s.set_budget(this->get_id_name(),mapper->get_time_step());
        //s.set_speed(this->get_id_name(),);

    }
private:
    void make_mapper(std::vector<APath> all_pathz, std::vector<double> porbablites_arr,std::vector<u_int16_t> l={})
    {
        mapper=std::make_unique<PathMapper<u_int32_t>>(std::move(all_pathz),std::move(porbablites_arr),l);
    }
    u_int64_t get_hash_state(const State<>& s)const {
        //cout<<s->to_string_state()<<endl;
        auto hSpeed = s.get_speed(this->my_id).hashConst(Point::maxSpeed);
        auto hPos = s.get_position_ref(this->my_id).hashConst();
        u_int64_t EntryIndx = Point::hashNnN(hPos,hSpeed);
        return EntryIndx;
    }
};


#endif //PE_STATICPOLICY_HPP
