//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATEDATA_HPP
#define PE_STATEDATA_HPP
#include "utils/game_util.hpp"
struct Single{
public:
    int x;
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
    vector<u_int32_t> x;
    Complex()=default;
    explicit Complex(vector<u_int32_t> &&vec):x(vec){}
    [[nodiscard]] u_int64_t hash_it() const{
        return this->x.front();
    }

    [[nodiscard]] string to_string() const{
        return std::to_string(x.front());
    }


};

#endif //PE_STATEDATA_HPP
