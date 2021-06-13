//
// Created by eranhe on 6/12/21.
//

#ifndef PE_MEMORYRTDP_HPP
#define PE_MEMORYRTDP_HPP

#include "States/State.hpp"
#include "Rewards.hpp"
#include "../TrajectoriesTree.hpp"

typedef float Cell;
typedef std::vector<Cell> Row;
typedef u_int64_t Entry;
class MemoryRtdp{

    Rewards R = Rewards::getRewards();
    std::unique_ptr<std::unordered_map<Entry,Row>> Qtable;
    std::vector<Point> id_to_point;
    std::function<u_int64_t (const State<> &s)> hash_func;
    Randomizer rand;
    u_int32_t agent_max_speed = 1;
    agentEnum my_id = agentEnum::D;
    TrajectoriesTree trajectories_tree;

public:
    explicit MemoryRtdp(int seed,TrajectoriesTree &&_trajectories_tree):Qtable(std::make_unique<std::unordered_map<Entry ,Row>>()),
    id_to_point(Point::getVectorActionUniqie()),rand(seed),trajectories_tree(_trajectories_tree)
    {
        hash_func=[](const State<> &ptrS){return ptrS.getHashValue();};
    }
    std::pair<Point,u_int64_t> get_argMAx(const State<> &s);
    u_int64_t get_entry(const State<> &s);
    void Q_table_add_row(const State<> &s,Entry key_entry);
    int to_closet_path_H(const State<> &s);
    void heuristic(const State<> &s, Entry entry_index);
    void set_value_matrix(Entry entry, size_t second_entry, Cell val);

    bool isInQ(Entry id_state);
    Cell get_max_val(const State<> &s);
    const Row &get_row_qTable(const State<> &s, Entry id_state);
};

std::pair<Point, u_int64_t> MemoryRtdp::get_argMAx(const State<> &s) {

    auto entry_id = this->get_entry(s);
    const Row& row = this->get_row_qTable(s,entry_id);
    auto index = arg_max_at_shuffle<Row>(row,rand.generator);
    Point action = id_to_point[index];
    return std::pair<Point, u_int64_t>(std::move(action),entry_id);
}

u_int64_t MemoryRtdp::get_entry(const State<> &s){
    this->hash_func(s);
}
inline bool MemoryRtdp::isInQ(Entry id_state)
{
    if(auto pos = Qtable->find(id_state);pos == Qtable->end())
        return false;
    return true;

}
const Row& MemoryRtdp::get_row_qTable(const State<> &s,Entry id_state)
{
    if(!isInQ(id_state)){
        Q_table_add_row(s,id_state);
    }
    return Qtable->operator[](id_state);
}

void MemoryRtdp::Q_table_add_row(const State<> &s,Entry key_entry) {

    Qtable->try_emplace(key_entry,27,1);
    heuristic(s,key_entry);

}

//
// Created by eranhe on 6/13/21.
//

#ifndef PE_HEURISTIC_HPP
#define PE_HEURISTIC_HPP



void MemoryRtdp::heuristic(const State<>& s,Entry entry_index)
{

    vector<State<>> vec_q;
    auto oldState = State(s);

    size_t ctr=-1;
    for (const auto &item_action : this->id_to_point)
    {
        ctr++;
        // apply action state and let the envirmont to roll and check the reward/pos
        Point actionCur = item_action;

        double val;
        bool isWall = oldState.applyAction(my_id,actionCur,agent_max_speed,oldState.jump);

        int step = to_closet_path_H(oldState);


        if (isWall)
            val = (this->R.WallReward)*std::pow(R.discountF,oldState.jump);
        else
            val=(this->R.CollReward)*std::pow(R.discountF,step);

        oldState.assignment(s,this->my_id);
        // insert to Q table
        this->set_value_matrix(entry_index,item_action.hashMeAction(Point::actionMax) ,val);
    }

}

int MemoryRtdp::to_closet_path_H(const State<> &s) {
    //return 0;
    return this->trajectories_tree.get_min_steps(s);
}

void MemoryRtdp::set_value_matrix(Entry entry, size_t second_entry, Cell val) {
    auto& vec = this->Qtable->at(entry);
    auto old = vec[second_entry];
    vec[second_entry]=val;
    //this->Qtable->operator[](entryState).operator[](action.hashMeAction(Point::actionMax))=val;

}

Cell MemoryRtdp::get_max_val(const State<> &s) {
    auto r = this->get_row_qTable(s,this->get_entry(s));
    return *std::max_element(r.begin(),r.end());
}

#endif //PE_HEURISTIC_HPP


#endif //PE_MEMORYRTDP_HPP
