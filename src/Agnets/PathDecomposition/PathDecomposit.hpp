//
// Created by eranhe on 6/23/21.
//

#ifndef PE_PATHDECOMPOSIT_HPP
#define PE_PATHDECOMPOSIT_HPP
#include "utils/game_util.hpp"
#include "Agnets/DP/RTDP.h"
#include "Game/Initializer.hpp"
#include "TableAgg.h"

typedef std::vector<AStar::StatePoint> EvaderPath;

class PathDecomposit{

    std::vector<EvaderPath> all_paths;
    std::vector<double> prob_all_paths;
    State<> s_state;
    StaticPolicy* evader;
    configGame conf;
    vector<std::unique_ptr<Table>> l_Q_table;


public:
    PathDecomposit(StaticPolicy *evader_ptr,configGame &conf_object,const State<>& _s_state);
    void single_train();
    void train();

    void all_train();

private:
    std::unique_ptr<RTDP> decompos_paths();
    void update_evader_paths(std::vector<EvaderPath>&& pathsz,vector<double>&& p);


};

PathDecomposit::PathDecomposit(StaticPolicy *evader_ptr,configGame &conf_object,const State<>& _s_state) : evader(evader_ptr),conf(conf_object),s_state(_s_state)
{
    all_paths = evader->get_copy_pathz();
    prob_all_paths  = evader->get_copy_probabilities();
}

std::unique_ptr<RTDP> PathDecomposit::decompos_paths()
{
    return Initializer::init_RTDP(conf,evader);

}



void PathDecomposit::update_evader_paths(std::vector<EvaderPath>&& pathsz,vector<double>&& p)
{
    evader->set_mapper(std::make_unique<PathMapper<uint32_t>>(std::move(pathsz),std::move(p)));
}

void PathDecomposit::train()
{
    if(conf.a==1)
        all_train();
    else
        single_train();
}

void PathDecomposit::all_train()
{
    std::unique_ptr<RTDP> pursurer_agent = decompos_paths();
    auto sim  = Emulator(pursurer_agent.get(),evader, std::move(State<>(s_state)),conf);
    sim.main_loop(2000000);

}

void PathDecomposit::single_train()
{
    std::unordered_map<u_int64_t ,State<>> map_dico;

    auto all_paths_ = evader->get_copy_pathz();
    auto prob_all_paths_  = evader->get_copy_probabilities();

    for(auto & p_path : all_paths)
    {
        update_evader_paths({p_path},{1.0});
        std::unique_ptr<RTDP> pursurer_agent = decompos_paths();
        auto sim  = Emulator(pursurer_agent.get(),evader, std::move(State<>(s_state)),conf);
        sim.main_loop(20000);
        std::unique_ptr<Table> Q = pursurer_agent->get_Q_tabel();
        cout<<"Q:"<<Q->size()<<endl;
        l_Q_table.push_back(std::move(Q));

        auto map_s = pursurer_agent->get_map_state();
        map_dico.insert(map_s.begin(),map_s.end());
    }
    update_evader_paths(std::move(all_paths_),std::move(prob_all_paths_));
    FinderH h_con = FinderH(conf.maxD,conf.h,this->evader->list_only_pos(),std::move(map_dico));
    cout<<"l_Q_table:"<<l_Q_table.size()<<endl;
    cout<<"l_Q_table:"<<l_Q_table.size()<<endl;
    std::unique_ptr<Qtable_> big = containerFixAggregator::agg_Q_tables(this->evader->get_copy_probabilities(),l_Q_table,h_con);
    cout<<"big>>"<<big->size()<<endl;

    std::unique_ptr<RTDP> pursurer_agent = decompos_paths();
    pursurer_agent->set_Q_table(std::move(big));
    auto sim  = Emulator(pursurer_agent.get(),evader, std::move(State<>(s_state)),conf);
    sim.main_loop(2000000);
}




#endif //PE_PATHDECOMPOSIT_HPP