//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATEDATA_HPP
#define PE_STATEDATA_HPP
#include <utility>

#include "utils/game_util.hpp"
#include "GoalRec//NodeGR.hpp"

struct Single{
public:

    NodeG* ptr= nullptr;

    Single()=default;

    [[nodiscard]] u_int64_t hash_it() const{
        return 0;
    }
    [[nodiscard]] string to_string() const{
        return "";
    }
    Single& operator=(NodeG* _ptr){return *this;}
};

struct Complex{
public:
    std::vector<NodeG*> ptr;
    //explicit Complex(NodeG *ptr_n):ptr(ptr_n){}
    Complex():ptr(0){}

    [[nodiscard]] u_int64_t hash_it() const{
        if(ptr.empty()) return 0;
        u_int64_t h=0;
        std::for_each(ptr.begin(),ptr.end(),[&h](NodeG* n){
            h+=n->hash_it();
        });
        return h;
    }

    [[nodiscard]] std::vector<u_int16_t> get_plans() const{
        if(this->ptr.size()==1) return ptr[0]->get_plans();
        std::vector<u_int16_t > l = ptr[0]->get_plans();;
        for (int i = 1; i < this->ptr.size(); ++i) {
            auto res = ptr[i]->get_plans();
            for(auto plan_idx: res)
                if(auto pos = std::find(l.begin(),l.end(),plan_idx);pos == l.end())
                {
                    l.push_back(plan_idx);
                }
        }
        return l;
    }

    [[nodiscard]] string to_string() const{
        if(ptr.empty()) return "";
        string h;
        std::for_each(ptr.begin(),ptr.end(),[&h](NodeG* n){
            h+=n->node_to_string()+" | ";
        });
        return h;
    }
    Complex& operator=(std::vector<NodeG*> _ptr){ptr=std::move(_ptr); return *this;}


};

#endif //PE_STATEDATA_HPP
