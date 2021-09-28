//
// Created by eranhe on 6/6/21.
//

#ifndef PE_PATHMAPPER_HPP
#define PE_PATHMAPPER_HPP
//
// Created by eranhe on 01/10/2020.
//

#include <utility>

#include "States/State.hpp"
#include "Search/Astar.hpp"
typedef AStar::StatePoint StatePoint;

template <typename K>
class PathMapper{
    std::vector<std::vector<StatePoint>> all_paths;
    std::vector<u_int16_t> paths_names;
    unsigned int size_pathz=0;
    std::vector<double> probabilities;
    u_int64_t time_t = 0;
    u_int16_t current_path=-1;
    u_int32_t step_counter=0;
    std::unordered_map<uint64_t,vector<K>> mapper_pathz;
    std::unordered_map<uint64_t,vector<std::pair<int,int>>> state_lookup;
    std::vector<u_int32_t> memo;
    bool is_constant=false;
    int path_const = 0;
    int mode=-1;
public:
    explicit PathMapper(vector<vector<StatePoint>> pathz,std::vector<double> probabilities_paths,const std::vector<u_int16_t> &path_name_)
            :all_paths(std::move(pathz)),size_pathz(all_paths.size())
            ,probabilities(std::move(probabilities_paths)){
        crate_maper();
        int max = all_paths[0].size();
        for(int i=1;i<size_pathz;++i)
        {
            if (max<all_paths[i].size()){
                max=all_paths[i].size();
            }

        }
        memo=std::vector<u_int32_t>(max);
        if (path_name_.empty())
            paths_names= range_n(all_paths.size());
        else
            std::copy(path_name_.begin(),path_name_.end(),std::back_inserter(paths_names));

    }
    u_int32_t get_all_paths_size(){return all_paths.size();}
    u_int32_t get_max_path_size()
    {
        u_int32_t max =0;
        for (auto path_item : all_paths)
        {
            if(max<path_item.size())
                max=path_item.size();
        }
        return max;
    }
    void set_constant(int k)
    {
        if(k>=0)
            is_constant=true;
        else
            is_constant=false;
        path_const = k;
    }
    PathMapper()=default;
    u_int get_choosen_path(){return this->current_path;}
    u_int32_t get_time_step(){return time_t;}
    void random_choose_path(double seed)
    {
        //assert(step_counter==0);
        double acc=0;
        u_int i=0;
        while (i<(size_pathz-1))
        {
            acc+=this->probabilities[i];
            if(acc>=seed)
                break;
            ++i;
        }
        this->current_path=i;
        if(is_constant)
            this->current_path=this->path_const;
        //this->current_path=22;
        this->time_t=0;
        step_counter=0;

        this->memo[step_counter]=time_t;
    }

//    vector<tuple<StatePoint,int,double>> get_next_states(u_int64_t hash_state,int jumps)
//    {
//        assert(step_counter>=0 );
//        time_t=memo[step_counter--];
//        std::vector<K> indexing_arr = this->mapper_pathz.at(hash_state);
//        double sum=0;
//        std::for_each(indexing_arr.begin(),indexing_arr.end(),[&](auto &i){sum+=this->probabilities[i];});
//        vector<tuple<StatePoint,int,double>> next_states_list;
//        next_states_list.reserve(indexing_arr.size());
//        std::for_each(indexing_arr.begin(),indexing_arr.end(),[&](const K& index_path){
//            next_states_list.emplace_back(get_jumping_state(time_t,index_path),time_t,this->probabilities[index_path]/sum);
//        });
//
//
//        return next_states_list;
//
//    }

    vector<tuple<StatePoint,int,double>> get_next_statesV2(u_int64_t hash_state,int jumps,int mode=0)
    {
        assert(step_counter>=0 );
        time_t=memo[step_counter--];
        assert(time_t-memo[step_counter]==jumps);
        std::vector<std::pair<int,int>> indexing_arr = this->state_lookup.at(hash_state);
        if (mode==2)
            indexing_arr = filter_only_exact_time(indexing_arr,memo[step_counter]);
        double sum=0;
        std::for_each(indexing_arr.begin(),indexing_arr.end(),[&](const std::pair<int,int> &i){sum+=this->probabilities[i.first];});
        vector<tuple<StatePoint,int,double>> next_states_list;
        next_states_list.reserve(indexing_arr.size());
        std::for_each(indexing_arr.begin(),indexing_arr.end(),[&](const std::pair<int,int> &item){

            int index_path = item.first;
            int s_t = item.second+jumps;
            next_states_list.emplace_back(get_jumping_state(s_t,index_path),s_t,this->probabilities[index_path]/sum);

        });

        return next_states_list;

    }

    auto filter_only_exact_time(const std::vector<std::pair<int,int>> &indexing_arr ,int t)
    {
        std::vector<std::pair<int,int>> results;
        for(int i=0;i<indexing_arr.size();++i)
        {
            if (indexing_arr[i].second==t)
                results.push_back(indexing_arr[i]);
        }
        return  results;
    }

    StatePoint get_next_actual_state(u_int jump=1)
    {
        assert(jump>0);

        time_t+=jump;
        step_counter++;
        //cout<<"step_counter"<<step_counter<<endl;
        assert(step_counter<5000);
        memo[step_counter]=time_t;

        return get_jumping_state(time_t,this->current_path);
    }
    [[nodiscard]] std::vector<std::vector<StatePoint>> get_all_pathz()const{return all_paths;}
    [[nodiscard]] const std::vector<std::vector<StatePoint>>& get_all_pathz_ref()const{return all_paths;}
    [[nodiscard]] vector<u_int16_t> get_names_pathz()const{return paths_names;}
    [[nodiscard]] std::vector<double> get_all_probabilites()const{return probabilities;}
    [[nodiscard]] const std::vector<double>& get_all_probabilites_ref()const{return probabilities;}
    [[nodiscard]] std::vector<std::vector<Point>> get_all_pos()const
    {
        std::vector<std::vector<Point>> l;
        l.reserve(all_paths.size());
        for(const auto &item:all_paths)
        {
            std::vector<Point> li;
            li.reserve(item.size());
            std::transform(item.begin(), item.end(), std::back_inserter(li), [](const StatePoint& s){
                return s.pos;
            });
            l.push_back(std::move(li));
        }
        return l;
    }

    int sanity_check(double seed)
    {
        this->random_choose_path(seed);
        cout<<"P:\t"<<this->current_path<<endl;
        return this->current_path;
    }

    StatePoint get_cur_position()
    {
        assert(time_t==0);
        return all_paths[this->current_path][time_t];
    }

private:

    void crate_maper()
    {

        for(int i=0;i<size_pathz;++i)
        {

            for (const auto &item:all_paths[i])
            {
                u_int64_t h= item.getHashStateAttacker();
                if(auto pos = mapper_pathz.find(h);pos==mapper_pathz.end())
                {
                    auto itesr = mapper_pathz.try_emplace(h,1);
                    itesr.first->second[0]=i;
                }
                else{
                    pos->second.push_back(i);
                }

            }
        }
        populate_dict_lookup();
        char sep=';';
        auto iter = probabilities.begin();
        int p_num=0;
        for(const auto& item:all_paths) {
            cout<<p_num++<<": "<<*(iter++)<<": "<<sep;
            int ctr_time=0;
            for (const auto & i : item) {
                cout <<ctr_time++<<":"<< i << sep;
            }
            cout<<"\n"<<endl;
        }

    }

    StatePoint get_jumping_state(u_int64_t idx_time,size_t idx_path)
    {
        auto t = std::min(idx_time,this->all_paths[idx_path].size()-1);
        return this->all_paths[idx_path][t];
    }


    std::vector<u_int16_t> range_n(u_int16_t x)
    {
        std::vector<u_int16_t > l;
        for( u_int16_t i = 0; i < x; i++ )
            l.push_back(i);
        return l;
    }

    void populate_dict_lookup()
    {
        for (int i=0;i<all_paths.size();++i)
        {
            for (int t_i=0;t_i<all_paths[i].size();++t_i)
            {
                auto item = all_paths[i][t_i];
                u_int64_t h= item.getHashStateAttacker();
                if(auto pos = this->state_lookup.find(h);pos==state_lookup.end())
                {
                    auto itesr =state_lookup.template try_emplace(h,1);
                    itesr.first->second[0]={i,t_i};
                }else{
                    pos->second.push_back({i,t_i});
                }
            }
        }
    }

};

#endif //PE_PATHMAPPER_HPP
