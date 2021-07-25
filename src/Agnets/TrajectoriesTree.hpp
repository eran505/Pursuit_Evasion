//
// Created by eranhe on 6/13/21.
//

#ifndef PE_TRAJECTORIESTREE_HPP
#define PE_TRAJECTORIESTREE_HPP
#define MAX_STEP 100000
#include "GoalRec/PathRec.hpp"
#include "States/State.hpp"

class TrajectoriesTree{

   // GoalRecognition GR;
    unordered_map<u_int64_t ,vector<int>> dict_mapper_pathz;
    Rewards R = Rewards::getRewards();
    agentEnum e=agentEnum::A;
    agentEnum p=agentEnum::D;
    std::vector<std::vector<Point>> all_paths;
    std::vector<double> prior_p_paths;
    [[maybe_unused]] std::vector<std::unique_ptr<NodeG>> tmp_lst;
    std::unique_ptr<std::unordered_map<u_int64_t,std::vector<int16_t>>>  dict_loc_time= nullptr;
    vector<u_int16_t > names;
public:

    explicit TrajectoriesTree(const std::vector<std::vector<Point>> &pathz,const std::vector<double>& probablities,std::vector<u_int16_t>&& names_):names(names_){
        dict_mapper_pathz=std::unordered_map<u_int64_t ,vector<int>>();
        dict_evader_paths(pathz);
        all_paths = pathz;
        fill_map_loc();
        prior_p_paths = probablities;
    }

    void fill_map_loc()
    {
        dict_loc_time = std::make_unique<std::unordered_map<u_int64_t,std::vector<int16_t>>>(evader_loc_time_t_to_pahts());
    }

    std::vector<std::vector<Point>> get_pathz(){return all_paths;}


    State<> deduce_belief_state(const State<> &s_state)
    {
        State<> s = State(s_state);
        auto loc_evader = s.get_position_ref(this->e);
        auto t_ = s.state_time;
        //cout<<"loc:"<<loc_evader.to_str()<<"\t t:"<<t_<<endl;
        auto v = this->dict_loc_time->at(loc_evader.expHash()+t_);
        //cout<<v<<endl;
        if (v.size()==1)
            return s;
        auto x =std::make_unique<NodeG>(s_state.budgets.ptr.front()->deep_copy_singel(s_state.budgets.ptr.front()));
        s.budgets.ptr.clear();
        s.budgets.ptr.push_back(x.get());
        tmp_lst.push_back(std::move(x));

        auto old_g = s.budgets.ptr.front()->goal_list;
        s.budgets.ptr.front()->goal_list.front().second.clear();
        for(auto i : v)
            s.budgets.ptr.front()->goal_list.front().second.push_back(i);

        return s;
    }



    double get_min_steps_all_end(const State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);

        auto begin_look_up = s.state_time+s.jump;
        auto vec = s.budgets.get_plans();
        u_int32_t min_step_val=100000;
        for (int index_path : vec) {
            auto idx = name_to_idx(index_path);
            if (auto dist= get_min_step_diff_last_orgin(p_pos,idx,begin_look_up);dist<min_step_val)
                min_step_val = dist;
        }
        auto h = this->R.CollReward*std::pow(R.discountF,min_step_val+s.state_time);
        return h;
    }

    double get_min_steps_all_endV2(const State<> &s)
    {
        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time+s.jump;
        u_int32_t min_step_val=1000000;
        for (int index_path : names) {
            auto idx = name_to_idx(index_path);
            if (auto dist= get_min_step_diff_last_orgin(p_pos,idx,begin_look_up);dist<min_step_val)
                min_step_val = dist;
        }
        return this->R.CollReward*std::pow(R.discountF,min_step_val);
    }



    double get_min_steps_rel_end(const State<> &s)const
    {

        const Point& evader_position = s.get_position_ref(e);
        const Point& p_pos = s.get_position_ref(this->p);

        auto begin_look_up = s.state_time+s.jump;
        auto vec = s.budgets.get_plans();
        u_int32_t min_step_val=100000;
        for (int index_path : vec) {
            auto idx = name_to_idx(index_path);
            if (auto dist= get_min_step_diff_last_orgin(p_pos,idx,begin_look_up);dist<min_step_val)
                min_step_val = dist;
        }
        auto h = this->R.CollReward*std::pow(R.discountF,min_step_val+begin_look_up);
        return h;
    }

    double all_future_distances_min(const State<>& s,bool is_optim=true)const
    {
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time;
        auto vec = s.budgets.get_plans();
        std::vector<u_int32_t>D(vec.size());
        for(int k=0;k<vec.size();k++)
        {
            u_int index_plan = name_to_idx(vec[k]);
            D[k]=get_min_step_diffV2(p_pos,index_plan,begin_look_up,is_optim);
        }
        return this->R.CollReward*std::pow(R.discountF,*std::min_element(D.begin(),D.end()));;
    }

    double all_future_distances_expection(const State<>& s,bool is_optim=true)const
    {
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time;
        //begin_look_up = s.state_time;
        auto vec = s.budgets.get_plans();
        std::vector<double>D(vec.size());
        double result=0.0;
        double sum_all_prob = 0.0;
        for(int k=0;k<vec.size();k++)
        {
            u_int index_plan = name_to_idx(vec[k]);
            auto steps =get_min_step_diffV2(p_pos,index_plan,begin_look_up,is_optim);
            if(steps==MAX_STEP) D[k] = 0;
            else D[k]= this->R.CollReward*std::pow(R.discountF,steps);
            sum_all_prob+=prior_p_paths[index_plan];
        }
        for (int i = 0; i < D.size(); ++i) {
            u_int index_plan = name_to_idx(vec[i]);
            result+=D[i]*(prior_p_paths[index_plan]/sum_all_prob);
        }
        //cout<<result<<endl;
        return result;
    }



    double H_planV2(const State<>& s,int plan_id,bool is_optim=true)const
    {
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time;
        auto step_to_coll=get_min_step_diffV2(p_pos,plan_id,begin_look_up,is_optim);
        double result =this->R.CollReward*std::pow(R.discountF,step_to_coll);
        return result;
    }
    double H_plan(const State<>& s,int plan_id)const
    {
        int t=0;
        const Point& p_pos = s.get_position_ref(this->p);
        auto begin_look_up = s.state_time;
        auto step_to_coll=get_min_step_diff_last_orgin(p_pos,plan_id,begin_look_up);
        double result =this->R.CollReward*std::pow(R.discountF,step_to_coll);
        return result;
    }

private:

    u_int32_t get_min_step_diffV2(const Point& p_loc,size_t index_path, u_int start_look,bool optimazer=true)const
    {
        start_look = std::min(size_t(start_look),all_paths[index_path].size());
        std::vector<u_int32_t > max_distance;
        int min_d = MAX_STEP;
        for(int i=start_look;i<all_paths[index_path].size()-1;i++)
        {
            auto res = Point::distance_min_step(p_loc,all_paths[index_path][i]);
            int t_e = int(i-start_look);
            if (optimazer)
            {
                if (res <= t_e)
                {
                    if(min_d>t_e) min_d = t_e;
                }
            }
        }
        return min_d;
    }

    u_int32_t get_min_step_diff_last_orgin(const Point& p_pos,size_t index_path, u_int start_look)const
    {
        u_int32_t min_step= Point::distance_min_step(p_pos,all_paths[index_path][start_look]);
        for (int i = int(start_look)+1; i < all_paths[index_path].size()-1 ; ++i) {
            auto res = Point::distance_min_step(p_pos,all_paths[index_path][i]);
            if (min_step>res) min_step=res;
        }
        return min_step;
    }

    std::unordered_map<u_int64_t,std::vector<int16_t>> evader_loc_time_t_to_pahts()
    {
        std::unordered_map<u_int64_t,std::vector<int16_t>> dico;
        for(int i=0;i<all_paths.size();++i){
            for (int j=0;j<all_paths[i].size();++j){
                u_int64_t h = all_paths[i][j].expHash()+j;
                //cout<<"loc:"<<all_paths[i][j].to_str()<<"\t t:"<<j<<endl;
                if(auto pos = dico.find(h);pos==dico.end()){
                    auto itesr = dico.try_emplace(h,1);
                    itesr.first->second[0]=i;
                }
                else{
                    pos->second.push_back(i);
                }
            }
        }
        return dico;
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
    long name_to_idx(u_int16_t x)const
    {
        auto pos = std::find(names.begin(),names.end(),x);
        return std::distance(names.begin(),pos);
    }


};

#endif //PE_TRAJECTORIESTREE_HPP
