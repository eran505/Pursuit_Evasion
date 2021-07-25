//
// Created by eranhe on 6/12/21.
//

#ifndef PE_MEMORYRTDP_HPP
#define PE_MEMORYRTDP_HPP

#include "States/State.hpp"
#include "Rewards.hpp"
#include "../TrajectoriesTree.hpp"
#include "Heuristicer.hpp"

typedef double Cell;
typedef std::vector<Cell> Row;
typedef u_int64_t Entry;
typedef std::unordered_map<Entry ,Row> Table;
class MemoryRtdp{

    //Rewards R = Rewards::getRewards();
    std::unique_ptr<std::unordered_map<Entry,Row>> Qtable;
    std::vector<Point> id_to_point;
    std::function<u_int64_t (const State<> &s)> hash_func;
    Randomizer rand;
    std::unordered_map<u_int64_t,State<>> debug_map;
    Heuristicer heuristicer;
    //std::vector<State<>> tml_list_to_del;
    bool verbose=false;

public:

    explicit MemoryRtdp(int seed,vector<vector<Point>> &&pathz_all,std::vector<double> &&prior_paths,vector<u_int16_t> &&names_paths,int mode,int option,int h,int max_speed):Qtable(std::make_unique<std::unordered_map<Entry ,Row>>()),
    id_to_point(Point::getVectorActionUniqie()),rand(seed),heuristicer(max_speed,h,pathz_all,prior_paths,std::move(names_paths)){
        init_object(mode,h);
    }

    MemoryRtdp(int seed,vector<vector<Point>> &&pathz_all,std::vector<double> &&prior_paths,vector<u_int16_t> &&names_paths,int mode,int option,int h,std::unique_ptr<Table> ptr_Q,int max_speed):Qtable(std::move(ptr_Q)),
    id_to_point(Point::getVectorActionUniqie()),rand(seed),heuristicer(max_speed,h,pathz_all,prior_paths,std::move(names_paths)){
        init_object(mode,h);
    }
    void init_object(int mode,int h)
    {

        if (mode == 0) hash_func = [](const State<> &ptrS) { return ptrS.getHashValue();};
        else if(mode==1) hash_func=[](const State<> &ptrS){return ptrS.getHashValueGR();};
        else if(mode==2) hash_func=[](const State<> &ptrS){return ptrS.getHashValueT();};
        else assert(false);
    }

    //[[maybe_unused]] auto retrun_list_h(){return this->tml_list_to_del;}


    std::unordered_map<u_int64_t,State<>> get_map_state(){return std::move(debug_map);}
    std::pair<Point,u_int64_t> get_argMAx(const State<> &s);
    u_int64_t get_entry(const State<> &s);
    void Q_table_add_row(const State<> &s,Entry key_entry);
    void set_value_matrix(Entry entry, size_t second_entry, Cell val);

    bool isInQ(Entry id_state);
    Cell get_max_val(const State<> &s);
    const Row &get_row_qTable(const State<> &s, Entry id_state);
    void set_Q_table(std::unique_ptr<Table> && t){this->Qtable=std::move(t);}
    std::unique_ptr<Table> get_Q_table();
    void set_print_mode(int mode)
    {
        verbose = mode==1;
    }
};

std::pair<Point, u_int64_t> MemoryRtdp::get_argMAx(const State<> &s) {
    auto entry_id = this->get_entry(s);
    const Row& row = this->get_row_qTable(s,entry_id);
    auto index = arg_max_at_shuffle<Row>(row,rand.generator);
    Point action = id_to_point[index];
    return std::pair<Point, u_int64_t>(std::move(action),entry_id);
}

u_int64_t MemoryRtdp::get_entry(const State<> &s){
    //return s.getHashValueGR();
    return this->hash_func(s);
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

    if(verbose) cout<<"[H] "<<s.to_string_state()<<endl;

//    auto s_tmp = State<>(s);
//    s_tmp.set_speed(agentEnum::A,Point(0));
//    if(s_tmp.to_string_state()=="2_A_(3, 11, 1, )_(0, 0, 0, )|D_(19, 9, 1, )_(-1, 0, 1, )|_{ 0}_N_(3, 11, 1, ) | _1" or
//            s_tmp.to_string_state()=="3_A_(7, 12, 2, )_(0, 0, 0, )|D_(19, 10, 1, )_(0, 1, 0, )|_{ 0}_N_(7, 12, 2, ) | _2")
//    {
//        tmp_var = heuristicer.heuristic(s);
//        int ctr=0;
//        for(auto item : tmp_var)
//            cout<<ctr++<<"="<<item<<";";
//        cout<<endl;
//    }else{
//        tmp_var = heuristicer.heuristic(s);
//    }
    auto x = heuristicer.heuristic(s);
    cout<<s.to_string_state()<<" : ";
    for (int i = 0; i < x.size(); ++i) {
        cout<<i<<":"<<x[i]<<" ";
    }
    cout<<endl;

    Qtable->try_emplace(key_entry,heuristicer.heuristic(s));
    debug_map.try_emplace(key_entry,s);


}



std::unique_ptr<Table> MemoryRtdp::get_Q_table(){

    return std::move(this->Qtable);

}



void MemoryRtdp::set_value_matrix(Entry entry, size_t second_entry, Cell val) {
    auto& vec = this->Qtable->at(entry);
    auto old = vec[second_entry];
//    if(!(old>=val))
//        cout<<old<<":->"<<val<<endl;
    //assert(old>=val);


    vec[second_entry]=val;
    //this->Qtable->operator[](entryState).operator[](action.hashMeAction(Point::actionMax))=val;

}

Cell MemoryRtdp::get_max_val(const State<> &s){
    auto r = this->get_row_qTable(s,this->get_entry(s));
    return *std::max_element(r.begin(),r.end());
}


#ifndef PE_HEURISTIC_HPP
#define PE_HEURISTIC_HPP

#endif //PE_HEURISTIC_HPP


#endif //PE_MEMORYRTDP_HPP
