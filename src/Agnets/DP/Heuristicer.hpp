//
// Created by eranhe on 6/27/21.
//

#ifndef PE_HEURISTICER_HPP
#define PE_HEURISTICER_HPP
#include "States/State.hpp"

typedef double Cell;
typedef u_int64_t Entry;

class Heuristicer{


    std::vector<Point> id_to_point=Point::getVectorActionUniqie();
    int max_speed_P;
    agentEnum Eid = agentEnum::A;

    agentEnum Pid = agentEnum::D;
    Rewards R = Rewards::getRewards();
    TrajectoriesTree trajectories_tree;
    std::function<double (const State<> &s)> H;
    std::function<double (const State<> &s,int plan)> H_plan;
    std::unordered_map<u_int64_t,vector<u_int32_t>> pos_to_path;

public:
    explicit Heuristicer(int maxP,int h,const vector<vector<Point>>& l_pathz,const vector<double> &prob,std::vector<u_int16_t> &&vec_names):max_speed_P(maxP),trajectories_tree(l_pathz,prob,std::move(vec_names))
    {
        //make_dict_pos_to_path_idx(l_pathz);

        if (h==0){
            H = [this](const State<> &ptrS) { return R.CollReward;};
            H_plan=[this](const State<> &ptrS,int index_plan) { return R.CollReward;};
        }
        if (h==1)
        {
            H = [this](const State<> &ptrS) { return this->trajectories_tree.all_future_distances_min(ptrS,true);};
            H_plan=[this](const State<> &ptrS,int index_plan) { return this->trajectories_tree.H_planV2(ptrS,index_plan,true);};
        }
        if (h==2)
        {
            H = [this](const State<> &ptrS) { return this->trajectories_tree.all_future_distances_expection(ptrS, true);};
            H_plan=[this](const State<> &ptrS,int index_plan) { return this->trajectories_tree.H_planV2(ptrS,index_plan,true);};
        }

        if (h==3)
        {
            H = [this](const State<> &ptrS) { return this->trajectories_tree.get_future_dist_all_paths(ptrS);};
            H_plan=[this](const State<> &ptrS,int index_plan) { return this->trajectories_tree.H_plan(ptrS,index_plan);};
        }
        if (h==4)
        {
            H = [this](const State<> &ptrS) { return this->trajectories_tree.get_future_dist_all_paths_imporve(ptrS);};
            H_plan=[this](const State<> &ptrS,int index_plan) { return this->trajectories_tree.H_plan(ptrS,index_plan);};
        }
        if (h==5)
        {
            H = [this](const State<> &ptrS) { return this->trajectories_tree.get_future_dist_all_paths_imporve_expection(ptrS);};
            H_plan=[this](const State<> &ptrS,int index_plan) { return this->trajectories_tree.H_plan(ptrS,index_plan);};
        }


    }
    void make_dict_pos_to_path_idx(const vector<vector<Point>>& l_pathz)
    {
        u_int32_t indx_id_path=-1;
        for (const auto &path :l_pathz)
        {
            indx_id_path++;
            for (const auto& pos:path)
            {
                auto ky = pos.expHash();
                if (auto pos_iter = this->pos_to_path.find(ky);pos_iter==pos_to_path.end()) {
                        auto item = pos_to_path.try_emplace(ky);
                        item.first->second.push_back(indx_id_path);

                } else{
                    cout<<"in"<<endl;
                    pos_iter->second.push_back(indx_id_path);
                }
            }
        }
    }

    [[nodiscard]] std::vector<Cell> heuristic(const State<>& s)const;

    [[nodiscard]] std::vector<Cell> heuristic_plan(const State<>& s,int plan)const;


    State<> infer_state( const State<> &s)
    {
       return trajectories_tree.deduce_belief_state(s);
    }
};

std::vector<Cell> Heuristicer::heuristic(const State<>& s)const
{
   // cout<<s.to_string_state()<<endl;
//    if(s.to_string_state()=="4_A_(8, 9, 1)_(2, -1, -1)|D_(14, 7, 3)_(1, -1, 0)|_[0]_N_(8, 9, 1) | _1")
//        cout<<"in"<<endl;
    vector<State<>> vec_q;
    auto oldState = State(s);
    auto row = std::vector<Cell>(27);
    size_t ctr=-1;
    for (const auto &item_action : this->id_to_point)
    {
        ctr++;
        Point actionCur = item_action;

        double val;
        bool isWall = oldState.applyAction(Pid,actionCur,max_speed_P,oldState.jump);
        //cout<<"oldState:\t"<<oldState.to_string_state()<<endl;

        if (isWall)
            val = (this->R.WallReward)*std::pow(R.discountF,oldState.jump);
        else{
            val = H(oldState);
        }
        oldState.assignment(s,this->Pid);
        row[item_action.hashMeAction(Point::actionMax)]=val;
    }
    return row;
}


std::vector<Cell> Heuristicer::heuristic_plan(const State<>& s,int plan)const
{

    vector<State<>> vec_q;
    auto oldState = State(s);
    auto row = std::vector<Cell>(27);
    size_t ctr=-1;
    for (const auto &item_action : this->id_to_point)
    {
        ctr++;
        Point actionCur = item_action;

        double val;
        bool isWall = oldState.applyAction(Pid,actionCur,max_speed_P,oldState.jump);

        if (isWall)
            val = (this->R.WallReward)*std::pow(R.discountF,oldState.jump);
        else{
            val = this->H_plan(oldState,plan);
        }
        oldState.assignment(s,this->Pid);
        row[item_action.hashMeAction(Point::actionMax)]=val;
    }
    return row;
}
#endif //PE_HEURISTICER_HPP
