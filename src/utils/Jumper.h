//
// Created by eranhe on 6/15/21.
//

#ifndef PE_JUMPER_H
#define PE_JUMPER_H
#include "GoalRec/PathRec.hpp"
static int get_step_number(int max);
static Point get_dif(const Point &e, const Point &p);
static int get_jumps(const Point &e, const Point &p);

namespace Jumper {
    static int get_step_number(int max) {
        if(max==0) return 1;
        auto d = std::max((int(log2(max)) + 1) - 3, 0);
        int res = std::pow(2, d);
        return res;
    }

    static Point get_dif(const Point &e, const Point &p) {
        return (e - p).AbsPoint();
    }

    static int get_jumps(const Point &e, const Point &p) {
        //return 1;
        int x = Jumper::get_step_number(Jumper::get_dif(e, p).getMax());
        //x = std::max(int(x/2),1);
        return x;
    }

    static int get_jumps_splits(const Point &e, const Point &p,int time,const std::vector<int> &splits)
    {

        if (Jumper::get_jumps(e,p)<3)
            return Jumper::get_jumps(e,p);
        auto split_n = splits[time];
        for (int i = 0; i < splits.size()-time; ++i) {
            if(split_n!=splits[i+time])
                return i;
        }
        return -1;
    }

}
#endif //PE_JUMPER_H
