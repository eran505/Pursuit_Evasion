//
// Created by eranhe on 6/26/21.
//

#ifndef PE_TABLEAGG_H
#define PE_TABLEAGG_H
#include "Agnets/DP/Heuristicer.hpp"
typedef float Cell;
typedef std::unordered_map<u_int64_t,std::vector<Cell>> Qtable_;
class FinderH{
public:
    Heuristicer heuristic;
    std::unordered_map<u_int64_t ,State<>> look_up_state;
    FinderH(int maxP_,int h_idx,const vector<vector<Point>>& l,
            std::unordered_map<u_int64_t ,State<>>&& dico):
            heuristic(maxP_,h_idx,l),look_up_state(std::move(dico)){}

    vector<Cell> find(u_int64_t state_id_)const
    {
        return heuristic.heuristic(look_up_state.at(state_id_));
    }

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
                      const vector<double>& pVec,const FinderH& h_con)
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

                func4(big, item.first, QVec, pVec, h_con,diff,same);
            }

        }
        cout<<"same: "<<same<<endl;
        cout<<"diff: "<<diff<<endl;
    }

    static auto agg_Q_tables(const vector<double>& pVec,const std::vector<std::unique_ptr<Qtable_>>& QVec,const FinderH& h_con)
    {
        assert(pVec.size()==QVec.size());
        u_int size_of_abstraction = QVec.front()->size();
        std::unique_ptr<Qtable_> big = std::make_unique<Qtable_>();
        func2(big.get(),QVec,pVec,h_con);
        cout<<"[ DONE ]"<<endl;
        return big;
    }
    template<typename T>
    static auto func3(const vector<T>& vec_i , const vector<T>& vec_big, double p){
        const auto _vecP = self_agg(vec_i,p);
        return agg(vec_big,_vecP);
    }
    static void func4(unordered_map<u_int64_t,vector<Cell>>* big, u_int64_t keyState, const std::vector<std::unique_ptr<Qtable_>>& QVec,
                      const vector<double>& pVec, const FinderH& h_con, uint& dif, uint& same){

        int occur =0;
        auto posBig = big->insert({keyState,vector<Cell>(27)}).first;
        vector<Cell> h_value = h_con.find(keyState);
        Cell max_item=1000000;
        //std::fill(h_value.begin(),h_value.end(),0);
        double p_h=0;
        for(size_t k=0;k<QVec.size();++k)
        {

            if(auto pos = QVec[k]->find(keyState);pos==QVec[k]->end())
            {
                p_h+=pVec[k];
            }
            else{
                const auto& vec_i = pos->second;
                max_item = std::min(max_item,*std::max_element(vec_i.begin(),vec_i.end()));
                posBig->second = func3(vec_i,posBig->second,pVec[k]);
                occur++;

            }
        }
//        for (int i = 0; i < h_value.size(); ++i) {
//            auto v_in = posBig->second[i]*1/(1-p_h);
//            if(h_value[i]>v_in) {
//                h_value[i] = v_in;
//            }
//        }



        posBig->second = func3(h_value,posBig->second,p_h);

    }

};


#endif //PE_TABLEAGG_H
