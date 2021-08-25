//
// Created by eranhe on 8/24/21.
//

#ifndef PE_BELIEFNODE_H
#define PE_BELIEFNODE_H
#include "utils/game_util.hpp"


class BeliefNode{

public:
    Point pos;
    int t=-1;
    int min_time_to_goal=-1;
    BeliefNode* father= nullptr;
    std::vector<std::unique_ptr<BeliefNode>> child_array{};
    vector<u_int32_t> beliefs{};
    vector<float> likelihood{};



    BeliefNode():pos(-1),child_array(std::vector<std::unique_ptr<BeliefNode>>()){};

    BeliefNode(const Point &p,int time,u_int32_t idx_path, float prior)
    :pos(p),
    t(time),
    child_array(std::vector<std::unique_ptr<BeliefNode>>()),
    beliefs(std::vector<u_int32_t>(1,idx_path)),
    likelihood(vector<float>(1,prior))
    {};

    void add_belief(int idx_path,float p)
    {
        this->beliefs.push_back(idx_path);
        this->likelihood.push_back(p);
    }
    string node_to_string()
    {
        string str="[";
        string sep=",";
        std::for_each(begin(beliefs), end(beliefs), [&](u_int32_t &piece){ str.append(std::to_string(piece)+sep);});
        str.append("]");
        str.append("_N_");
        str.append(this->pos.to_str());
        return str;
    }
    [[nodiscard]] u_int64_t hash_it() const {
        u_int64_t hash_number = pos.expHash()+t;
        for(const auto& path_id : beliefs){
            hash_number ^=  ((path_id+10) * 2654435761) + 2654435769 + (hash_number << 6) + (hash_number >> 2);
        }
        //hash_number ^=  ((t) * 2654435761) + 2654435769 + (hash_number << 6) + (hash_number >> 2);
        return hash_number;
    }

    [[nodiscard]] std::vector<u_int32_t> get_plans()const{return beliefs;}

    BeliefNode* search_child(const Point &loc) {
        for (auto & j : this->child_array) {
            if(j->pos==loc)
                return j.get();
        }
        return nullptr;
    }
    BeliefNode*  add_child(std::unique_ptr<BeliefNode> && ptr) {
        BeliefNode* node = ptr.get();
        this->child_array.push_back(std::move(ptr));
        return node;

    }


};













#endif //PE_BELIEFNODE_H
