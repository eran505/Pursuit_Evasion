//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATEDATA_HPP
#define PE_STATEDATA_HPP
#include "utils/game_util.hpp"
struct Single{
public:
    int x=0;
    explicit Single(int val): x(val){}
    Single()=default;
    [[nodiscard]] u_int64_t hash_it() const{
        return this->x;
    }
    [[nodiscard]] string to_string() const{
        return std::to_string(x);
    }

};

struct Complex{
public:
    vector<int> vec_data;
    Complex()=default;
    explicit Complex(vector<int> &&vec):vec_data(vec){}
    [[nodiscard]] u_int64_t hash_it() const{
        u_int32_t res = std::accumulate(vec_data.begin(),vec_data.end(),0);
        return res;
    }

    [[nodiscard]] string to_string() const{
        string str;
        std::for_each(vec_data.begin(),vec_data.end(),[&str](u_int32_t x){
            str+="_"+std::to_string(x);
        });
        return str;
    }



};

#endif //PE_STATEDATA_HPP
