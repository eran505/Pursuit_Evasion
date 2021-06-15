//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATEDATA_HPP
#define PE_STATEDATA_HPP
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
    NodeG* ptr;
    explicit Complex(NodeG *ptr_n):ptr(ptr_n){}
    Complex():ptr(nullptr){}

    [[nodiscard]] u_int64_t hash_it() const{
        if(!ptr) return 0;
        return ptr->hash_it();
    }

    [[nodiscard]] string to_string() const{
        if(!ptr) return "";
        return ptr->node_to_string();
    }
    Complex& operator=(NodeG* _ptr){ptr=_ptr; return *this;}


};

#endif //PE_STATEDATA_HPP
