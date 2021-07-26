//
// Created by eranhe on 6/26/21.
//

#ifndef PE_TABLEAGG_H
#define PE_TABLEAGG_H
#include "Agnets/DP/Heuristicer.hpp"
typedef double Cell;
typedef std::unordered_map<u_int64_t,std::vector<Cell>> Qtable_;
class FinderH{
public:
    Heuristicer heuristic;
    std::unordered_map<u_int64_t ,State<>> look_up_state;
    FinderH(int maxP_,int h_idx,const vector<vector<Point>>& l, std::vector<double> &&p,std::vector<u_int16_t >&& names_paths
            ,std::unordered_map<u_int64_t ,State<>>&& dico):
            heuristic(maxP_,h_idx,l,p,std::move(names_paths)),look_up_state(std::move(dico)){}

    vector<Cell> find(u_int64_t state_id_)const
    {
        return heuristic.heuristic(look_up_state.at(state_id_));
    }

    vector<Cell> find_plan(u_int64_t state_id_,int plan_idx)const
    {
        return heuristic.heuristic_plan(look_up_state.at(state_id_),plan_idx);
    }
    std::unordered_map<u_int64_t ,State<>> rtrun_dico_map(){return this->look_up_state;}
    auto infer_state(Qtable_* l_tabel)
    {
        Qtable_ t;
        for (auto &entry:*l_tabel) {
            const auto s = look_up_state.at(entry.first);

            auto state_new_s = heuristic.infer_state(s);
            u_int64_t new_s_hash = state_new_s.getHashValueGR();
            if(new_s_hash ==entry.first)
                continue;
//            cout<<"[OLD] "<<s.to_string_state()<<endl;
//            cout<<"[NEW] "<<state_new_s.to_string_state()<<endl;
//            cout<<"----\n";
            t.emplace(new_s_hash,entry.second);
            assert(l_tabel->find(new_s_hash)==l_tabel->end());
            look_up_state.try_emplace(new_s_hash,std::move(state_new_s));
        }
        l_tabel->insert(t.begin(),t.end());

    }
    bool is_one_plan(u_int64_t state_id_)const
    {
        for (auto& item: look_up_state.at(state_id_).budgets.ptr)
            for(auto &pair_item_goal:item->goal_list)
                if(pair_item_goal.second.size()>1)
                    return false;
        return true;
    }

    auto get_plans(u_int64_t state_id_)const
    {
        vector<u_int16_t> l;
        l.reserve(1);
        if (auto pos_iter = look_up_state.find(state_id_);pos_iter==look_up_state.end())
            assert(false);
        else{
            auto s = pos_iter->second;
            for (auto& item: s.budgets.ptr)
                for(auto &pair_item_goal:item->goal_list)
                    for(auto& planI : pair_item_goal.second)
                        l.push_back(planI);
        }

        return l;
    }

    auto get_state_from_id(u_int64_t state_id_)const
    {
        return look_up_state.at(state_id_) ;
    }
    std::unordered_map<u_int64_t ,State<>>&& get_map_dico(){return std::move(look_up_state);}

};


class containerFixAggregator{

public:

    template <typename T>
    static std::vector<T> agg(const std::vector<T>& a, const std::vector<T>& b)
    {
        assert(a.size() == b.size());

        std::vector<T> result;

        result.reserve(a.size());

        std::transform(a.begin(), a.end(), b.begin(),
                       std::back_inserter(result), [&](const auto& x1,const auto& x2){return x1+x2;});

        return result;
    }

    template <typename T>
    static std::vector<T> self_agg(const std::vector<T>& a,double p)
    {
        std::vector<T> result;

        result.reserve(a.size());

        std::transform(a.begin(), a.end(),
                       std::back_inserter(result), [&](const auto& x2){return p*x2;});

        return result;
    }

    static void func2(Qtable_* big,const std::vector<std::unique_ptr<Qtable_>>& QVec,
                      const vector<double>& pVec,const FinderH& h_con,bool is_plan_rec)
    {
        uint same=0;
        uint diff=0;

        for(size_t j=0;j<pVec.size();++j)
        {
            //cout<<"j:"<<j<<endl;
            for(const auto& item: *QVec[j])
            {

                if(big->find(item.first)!=big->end())
                    continue; // if the key is already in the table

                func4(big, item.first, QVec, pVec, h_con,diff,same,  is_plan_rec);
            }

        }
        cout<<"same: "<<same<<endl;
        cout<<"diff: "<<diff<<endl;
    }

    static auto agg_Q_tables(const vector<double>& pVec,const std::vector<std::unique_ptr<Qtable_>>& QVec,const FinderH& h_con,bool is_plan_rec)
    {
        assert(pVec.size()==QVec.size());
        u_int size_of_abstraction = QVec.front()->size();
        std::unique_ptr<Qtable_> big = std::make_unique<Qtable_>();
        func2(big.get(),QVec,pVec,h_con, is_plan_rec);
        cout<<"[ DONE ]"<<endl;
        return big;
    }
    template<typename T>
    static auto func3(const vector<T>& vec_i , const vector<T>& vec_big, double p){
        const auto _vecP = self_agg(vec_i,p);
        return agg(vec_big,_vecP);
    }
    static void func4(unordered_map<u_int64_t,vector<Cell>>* big, u_int64_t keyState, const std::vector<std::unique_ptr<Qtable_>>& QVec,
                      const vector<double>& pVec, const FinderH& h_con, uint& dif, uint& same,bool is_plan_rec){


        auto posBig = big->insert({keyState,vector<Cell>(27)}).first;
        bool is_one_plan = h_con.is_one_plan(keyState);
        vector<u_int16_t> plansIds = h_con.get_plans(keyState);
        //Cell max_item=1000000;
        State<> s = h_con.get_state_from_id(keyState);
        if (keyState==5662437437142756322)
            return;
        if(is_one_plan and is_plan_rec)
        {
            auto pos = QVec[plansIds.front()]->find(keyState);
            const auto& vec_i = pos->second;
            posBig->second = func3(vec_i,posBig->second,1.0);
            return;
        }

        if(is_plan_rec)
        {
            double sum_all_p = 0.0 ;
            for(auto plan_ix:plansIds) sum_all_p+=pVec[plan_ix];


            for(int j=0;j<plansIds.size();j++)
            {
                int plan_idx = plansIds[j];
                if(auto pos = QVec[plan_idx]->find(keyState);pos==QVec[plan_idx]->end())
                {
                    auto h_value =   h_con.find_plan(keyState,plan_idx);
                    posBig->second = func3(h_value,posBig->second,pVec[plan_idx]/sum_all_p);

                }else{
                    const auto& vec_i = pos->second;
                    posBig->second = func3(vec_i,posBig->second,pVec[plan_idx]/sum_all_p);
                }

            }

            return;
        }


        vector<Cell> h_value = h_con.find(keyState);

        double p_h=0;

        for(size_t k=0;k<QVec.size();++k)
        {

            if(auto pos = QVec[k]->find(keyState);pos==QVec[k]->end())
            {
                p_h+=pVec[k];
            }
            else{
                const auto& vec_i = pos->second;
                posBig->second = func3(vec_i,posBig->second,pVec[k]);

            }
        }
//        if(p_h<=0.25){
//            cout<<s.to_string_state()<<"\tPh:"<<p_h<<" ctr:"<<dif<<endl;
//            dif++;
//        }
        for (int i = 0; i < h_value.size(); ++i) {
            auto v_in = posBig->second[i]*1/(1-p_h);
            if(h_value[i]>v_in) h_value[i] = v_in;
        }



        posBig->second = func3(h_value,posBig->second,p_h);

    }

};


#endif //PE_TABLEAGG_H
