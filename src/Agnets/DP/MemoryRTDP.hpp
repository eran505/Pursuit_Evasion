//
// Created by eranhe on 6/12/21.
//

#ifndef PE_MEMORYRTDP_HPP
#define PE_MEMORYRTDP_HPP

#include "States/State.hpp"
typedef float Cell;
class MemoryRtdp{

    std::unique_ptr<std::unordered_map<u_int64_t,Cell>> Qtable;
    std::vector<Point> id_to_point;

public:
    MemoryRtdp():Qtable(std::make_unique<std::unordered_map<u_int64_t,Cell>>()),
    id_to_point(Point::getVectorActionUniqie()){

    }

    std::pair<Point,u_int64_t> get_argMAx(const State<> &s);
};

std::pair<Point, u_int64_t> MemoryRtdp::get_argMAx(const State<> &s) {
    return std::pair<Point, u_int64_t>();
}

#endif //PE_MEMORYRTDP_HPP
