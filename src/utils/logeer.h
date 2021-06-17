//
// Created by eranhe on 6/12/21.
//

#ifndef PE_LOGEER_H
#define PE_LOGEER_H
#include <utility>

#include "utils/game_util.hpp"
#include "fileIO/saver.hpp"
#include "fileIO/processerCSV.hpp"
class Logger{
    std::vector<u_int32_t> info;
    std::vector<u_int32_t> history;
    u_int64_t counter=0;
    std::string home;
    Saver<string> file_manger;
    u_int32_t log_every=1000;
    bool done = false;
public:
    enum info_val {ITER,COLL,GOAL,WALL};
    explicit Logger(const configGame& conf):info(4),history(4),home(conf.home),
    file_manger(conf.home+LOG_DIR+std::to_string(conf._seed)+"_u"+conf.idNumber+"_L"+std::to_string(conf.eval_mode)+"_A"+std::to_string(conf.alpha)+"_Eval.csv",10)
    {
        file_manger.set_header_vec({"episodes","Collision","Wall" ,"Goal"});
    }

    bool get_done()const{return done;};

public:
    void log_scalar_increment(info_val val){
        info[val]++;
        history[val]++;
        history[ITER]++;
        info[ITER]++;
        if (info[ITER]%log_every==0){
            counter++;
            print();
            flush();
        }
    }

    const vector<u_int32_t>& get_log_vector()
    {
        return info;
    }
    void print()const
    {
        cout<<"Iter:"<<info[ITER]*counter;
        cout<<"\tColl:"<<info[COLL];
        cout<<"\tGoal:"<<info[GOAL];
        cout<<"\tWall:"<<info[WALL];
        cout<<endl;
    }
    u_int32_t get_log_every()const{return log_every;}
private:
    void flush()
    {
        is_done();
        info[ITER]*=counter;
        file_manger.inset_data(info);
        file_manger.inset_endLine();
        //if(info[COLL]==log_every) is_optimal = true;
        info.assign(info.size(),0);
        //return is_optimal;
    }
    void is_done(){done = info[COLL]==log_every;}
};

#endif //PE_LOGEER_H
