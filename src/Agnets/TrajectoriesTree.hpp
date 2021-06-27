//
// Created by eranhe on 6/13/21.
//

#ifndef PE_TRAJECTORIESTREE_HPP
#define PE_TRAJECTORIESTREE_HPP

#include "GoalRec/PathRec.hpp"
#include "States/State.hpp"

class TrajectoriesTree{

   // GoalRecognition GR;
    unordered_map<u_int64_t ,vector<int>> dict_mapper_pathz;

    agentEnum e=agentEnum::A;
    agentEnum p=agentEnum::D;
    std::vector<std::vector<Point>> all_paths;
public:

    explicit TrajectoriesTree(const std::vector<std::vector<Point>> &pathz){
        dict_mapper_pathz=std::unordered_map<u_int64_t ,vector<int>>();
        dict_evader_paths(pathz);
        all_paths = pathz;
    }

    std::vector<std::vector<Point>> get_pathz(){return all_paths;}

    u_int32_t get_min_steps_next_foucs(const State<> &s)
    {
        //cout<<"[H] "<<s.to_string_state()<<endl;
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time+s.jump;
        auto vec = dict_mapper_pathz.at(evader_position.expHash());
        //cout<<vec<<endl;
        std::vector<u_int32_t> min_step;
        for (auto index_path: vec)
        {
            min_step.push_back(get_min_step_diff(p_pos,index_path,begin_look_up)/3);
        }
        assert(!min_step.empty());
        return u_int32_t(*std::min_element(min_step.begin(),min_step.end()));
    }


    u_int32_t get_min_steps_next_all(const State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time+s.jump;

        std::vector<u_int32_t> min_step;
        for (int index_path = 0; index_path < all_paths.size(); ++index_path) {

            min_step.push_back(get_min_step_diff(p_pos,index_path,begin_look_up)/3);
        }
        assert(!min_step.empty());
        return u_int32_t(*std::min_element(min_step.begin(),min_step.end()));
    }

    u_int32_t get_min_steps_all_end(const State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time+s.jump;

        std::vector<u_int32_t> min_step;
        for (int index_path = 0; index_path < all_paths.size(); ++index_path) {

            min_step.push_back(get_min_step_diff_last_orgin(p_pos,index_path,begin_look_up));
        }
        assert(!min_step.empty());
        return *std::min_element(min_step.begin(),min_step.end());
    }
    u_int32_t get_min_steps_rel_end(const State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time+s.jump;
        auto vec = dict_mapper_pathz.at(evader_position.expHash());

        std::vector<u_int32_t> min_step;
        for (int index_path : vec) {

            min_step.push_back(get_min_step_diff_last_orgin(p_pos,index_path,begin_look_up));
        }
        assert(!min_step.empty());
        return *std::min_element(min_step.begin(),min_step.end());
    }




private:

    u_int32_t get_min_step_diff(const Point& p_pos,size_t index_path, u_int start_look)
    {
        u_int32_t min_step=1000000;
        for (int i = start_look; i < start_look+1 ; ++i) {
            auto res = Point::distance_min_step(p_pos,all_paths[index_path][i]);
            if (min_step>res) min_step=res;
        }

        return min_step;
    }

    u_int32_t get_min_step_diff_last_orgin(const Point& p_pos,size_t index_path, u_int start_look)
    {
        u_int32_t min_step=1000000;
        for (int i = start_look; i < all_paths[index_path].size() ; ++i) {
            auto res = Point::distance_min_step(p_pos,all_paths[index_path][i]);
            if (min_step>res) min_step=res;
        }
        return min_step;
    }

    void dict_evader_paths(const std::vector<std::vector<Point>> &pathz)
    {
        for(int i=0;i<pathz.size();++i){
            for (const auto &item:pathz[i]){
                u_int64_t h = item.expHash();
                if(auto pos = dict_mapper_pathz.find(h);pos==dict_mapper_pathz.end()){
                    auto itesr = dict_mapper_pathz.try_emplace(h,1);
                    itesr.first->second[0]=i;
                }
                else{
                    pos->second.push_back(i);
                }
            }
        }
    }


};

#endif //PE_TRAJECTORIESTREE_HPP
