//
// Created by eranhe on 6/14/21.
//

#ifndef PE_NODEGR_HPP
#define PE_NODEGR_HPP
#include "utils/game_util.hpp"

struct NodeG{
    Point pos;
    NodeG* parent=nullptr;
    vector<std::unique_ptr<NodeG>> child;
    vector<pair<Point,std::vector<u_int16_t>>> goal_list;
    vector<double> goal_likelihood;

    int min_step=std::numeric_limits<int>::max();;
public:
    NodeG():pos(0),parent(nullptr),child(vector<std::unique_ptr<NodeG>>()),
            goal_list(vector<pair<Point,std::vector<u_int16_t>>>()){}
    explicit NodeG(const Point& p,const Point& g,u_int16_t id_path,double likelihood):pos(p),parent(nullptr),child(vector<std::unique_ptr<NodeG>>()),
                                                                                      goal_list(vector<pair<Point,std::vector<u_int16_t>>>()){
        this->goal_list.push_back({g,{id_path}});
        this->goal_likelihood.push_back(likelihood);
    }

//    explicit NodeG(const NodeG* ptr)
//    {
//        return NodeG(*ptr);
//    }
    NodeG deep_copy_singel(const NodeG* ptr)
    {
        return NodeG(ptr->pos,ptr->goal_list.front().first,ptr->goal_list.front().second.front(),ptr->goal_likelihood.front());
    }
    [[nodiscard]] u_int64_t hash_it()const
    {
        //u_int64_t hash_number = pos.hashConst();
        u_int64_t hash_number = pos.expHash();
        for (const auto& item : goal_list) {
            for(const auto& path_id : item.second){
                hash_number ^=  ((path_id+10) * 2654435761) + 2654435769 + (hash_number << 6) + (hash_number >> 2);
            }
        }
        return hash_number;
    }

    template<typename P = Point>
    void add_goal(P&& g,u_int16_t id_path,double likelihood)
    {
        auto it = std::find_if( goal_list.begin(), goal_list.end(),
                                [&g](const std::pair<Point, std::vector<u_int16_t>>& element){ return element.first == g;} );

        if (it==goal_list.end()) {
            goal_list.push_back({std::forward<P>(g), {id_path}});
            goal_likelihood.push_back(likelihood);

        }
        else {
            it->second.push_back(id_path);
            auto index_number = std::distance(goal_list.begin(), it);
            assert(index_number>=0);
            goal_likelihood[index_number]+=likelihood;
            assert(goal_likelihood[index_number]>=0 and goal_likelihood[index_number]<=1 );

        }
    }

    string node_to_string()
    {
        string str;
        for(const auto& item: goal_list)
        {
            str.append("[");
            std::for_each(item.second.begin(),item.second.end()-1,[&str](u_int16_t x){
                str.append(std::to_string(x)+",");
            });
            str.append(std::to_string(item.second.back()));
            str.append("]");
        }

        str.append("_N_");
        str.append(this->pos.to_str());
        return str;
    }

    [[nodiscard]] std::vector<u_int16_t > get_plans()const{
        std::vector<u_int16_t > ans;
        for (const auto &item :this->goal_list) {
            for (auto planz: item.second)
                ans.push_back(planz);
        }
        return ans;
    }

};
#endif //PE_NODEGR_HPP
