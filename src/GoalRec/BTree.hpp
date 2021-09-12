//
// Created by eranhe on 8/24/21.
//

#ifndef PE_BTREE_HPP
#define PE_BTREE_HPP
#include "utils/game_util.hpp"
#include "BeliefNode.h"
#define MAX_PATH_LEN 10000

class BTree{

    std::vector<std::vector<Point>> all_pathz;
    std::unique_ptr<BeliefNode> root;
    std::vector<double> probabilities;
    std::default_random_engine rng;
    int MAX_SPEED_E;
    int MAX_SPEED_P;
    unordered_map<u_int32_t,BeliefNode*> belief_dict;

public:
    int path_evader=0;
    BTree()=default;

    explicit BTree(int seed_, int E_speed, int P_speed):
    MAX_SPEED_E(E_speed),
    MAX_SPEED_P(P_speed),
    rng(seed_),
    root(std::make_unique<BeliefNode>())
    {};
    void calc_min_time_to_goal();
    void load_agent_paths(const std::vector<std::vector<Point>> &pathz,vector<double> &&path_probabilties,const vector<u_int16_t>& paths_name);
    BeliefNode* get_father_node(int t,int idx_path);
    void insert_belief_state_map(BeliefNode* n, int idx_path,int t);
    void print_path(int idx);
    vector<BeliefNode*> get_all_successor(BeliefNode *n,int steps)const;
    BeliefNode* get_next_step_by_path(BeliefNode *n, int steps,u_int path_id);
    BeliefNode* get_next_step_by_loc(BeliefNode *n, int steps,const Point &loc_e);
    static void assert_path_sizes(const vector<std::vector<Point>> &pathz);
    BeliefNode* get_start_ptr(const Point& l_e);
};

void BTree::print_path(int idx) {

    int max_len = all_pathz[idx].size();
    for (int i = 0; i < max_len; ++i) {
        u_int32_t entry = idx*MAX_PATH_LEN+i;
        auto node = this->belief_dict.at(entry);
        cout<<node->node_to_string();
        cout<<"  child:  ";
        for(const auto& x: node->child_array )
        {
            cout<<x->node_to_string();
            cout<<"  -  ";
        }
        cout<<endl;

    }
    cout<<"--------"<<endl;

}

void BTree::insert_belief_state_map(BeliefNode *n, int idx_path, int t) {
    this->belief_dict.try_emplace(MAX_PATH_LEN*idx_path+t,n);

}

BeliefNode *BTree::get_father_node(int t, int idx_path) {
    if(t<0) return this->root.get();
    BeliefNode* b_state = belief_dict.at(MAX_PATH_LEN*idx_path+t);
    return b_state;
}
void BTree::assert_path_sizes(const vector<std::vector<Point>> &pathz)
{
    for(const auto& item:pathz)
        assert(MAX_PATH_LEN>item.size());
}
void BTree::load_agent_paths(const vector<std::vector<Point>> &pathz,
                             vector<double> &&path_probabilties,
                             const vector<u_int16_t> &paths_name) {

    assert_path_sizes(pathz);
    probabilities = std::move(path_probabilties);
    all_pathz = pathz;
    bool done=false;
    int ctr=0;

    while (!done)
    {
        int num_skip=0;

        for (int i = 0; i < pathz.size(); ++i) {
            if(ctr>=pathz[i].size())
            {
                num_skip++;
                continue;
            }
            BeliefNode* curr_ptr_level_ = get_father_node(ctr-1,paths_name[i]);
            auto results_node =  curr_ptr_level_->search_child(pathz[i][ctr]);
            if (results_node== nullptr) {
                //add new node
                auto node = curr_ptr_level_->add_child(
                        std::make_unique<BeliefNode>(pathz[i][ctr], ctr, paths_name[i], probabilities[i]));
                insert_belief_state_map(node, paths_name[i], ctr);

            }
            else{
                results_node->add_belief(paths_name[i],probabilities[i]);
                insert_belief_state_map(results_node, paths_name[i], ctr);
            }
        }
        if (num_skip==pathz.size())
            done=true;
        ctr++;
    }
    calc_min_time_to_goal();
    //    for (int k=0;k<all_pathz.size();++k)
//        print_path(k);
    cout<<"[done]"<<endl;
}

vector<BeliefNode *> BTree::get_all_successor(BeliefNode *node,int steps) const{
    std::vector<BeliefNode*> results;
    auto all_belief = node->get_plans();
    auto t = node->t;
    for (auto b :all_belief)
    {
        auto next_t = std::min(int(t+steps),int(all_pathz[b].size()-1));
        auto entry = MAX_PATH_LEN*b+next_t;
        results.push_back(this->belief_dict.at(entry));
    }
    return results;
}

BeliefNode * BTree::get_next_step_by_path(BeliefNode *n, int steps,u_int path_id) {
    auto next_t = std::min(int(n->t+steps),int(all_pathz[path_id].size()-1));
    auto entry = MAX_PATH_LEN*path_id+next_t;
    return this->belief_dict.at(entry);

}

void BTree::calc_min_time_to_goal() {

    std::for_each(std::begin(this->belief_dict),std::end(this->belief_dict),
                  [&](const std::pair<u_int32_t,BeliefNode*> item )
    {
         const auto beliefs = item.second->get_plans();
         auto min=MAX_PATH_LEN;
         for (auto b : beliefs){
             auto to_end = all_pathz[b].size()-item.second->t;
             if(min>to_end) min = to_end;
         }
        item.second->min_time_to_goal=min;
    });

}

BeliefNode *BTree::get_start_ptr(const Point &l_e) {
    return root->search_child(l_e);
}

BeliefNode *BTree::get_next_step_by_loc(BeliefNode *node, int steps, const Point &loc_e) {
//    cout<<"["<<steps<<"] "<<node->node_to_string()<<"  loc:"<<loc_e.to_str()<<"\tP:"<<this->path_evader<<'\n';
    std::vector<BeliefNode*> results;
    auto all_belief = node->get_plans();
    auto t = node->t;
    for (auto b :all_belief)
    {
        b=path_evader;
        auto next_t = std::min(int(t+steps),int(all_pathz[b].size()-1));
        auto entry = MAX_PATH_LEN*b+next_t;
        auto candidate = this->belief_dict.at(entry);
        if (candidate->pos == loc_e) return candidate;
    }
    assert(false);
}


#endif //PE_BTREE_HPP
