//
// Created by eranhe on 6/12/21.
//

#ifndef PE_LOGEER_H
#define PE_LOGEER_H
#include "utils/game_util.hpp"

class Logger{
    std::vector<u_int32_t> info;
    u_int64_t counter=0;
public:
    enum info_val {COLL,GOAL,WALL};


    Logger():info(std::vector<uint32_t>(4)){}


public:
    inline void log_scalar_increment(info_val val){ info[val]++;counter++; }

    void print()
    {
        cout<<"["<<counter<<"]";
        cout<<"\t Coll:"<<info[COLL];
        cout<<"\t Goal:"<<info[GOAL];
        cout<<"\t Wall:"<<info[WALL];
        cout<<endl;
    }
};

#endif //PE_LOGEER_H
