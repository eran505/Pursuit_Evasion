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
    u_int32_t log_every=100;
public:
    enum info_val {ITER,COLL,GOAL,WALL};
    explicit Logger(const configGame& conf):info(5),history(5),home(conf.home),
    file_manger(conf.home+LOG_DIR+std::to_string(conf._seed)+"_u"+conf.idNumber+"_L"+std::to_string(conf.eval_mode)+"_A"+std::to_string(conf.alpha)+"_Eval.csv",10)
    {
        file_manger.set_header_vec({"episodes","Collision","Wall" ,"Goal"});
        cout<<"in"<<endl;
    }


public:
    void log_scalar_increment(info_val val){
        info[val]++;
        history[val]++;
        history[ITER]++;
        info[ITER]++;
        if (info[ITER]%log_every==0){
            counter++;
            flush();
        }
    }

    void print()
    {
        cout<<"\t Iter:"<<history[ITER];
        cout<<"\t Coll:"<<history[COLL];
        cout<<"\t Goal:"<<history[GOAL];
        cout<<"\t Wall:"<<history[WALL];
        cout<<endl;
    }

private:
    void flush()
    {
        info[ITER]*=counter;
        file_manger.inset_data(info);
        file_manger.inset_endLine();
        info.assign(info.size(),0);
    }
};

#endif //PE_LOGEER_H
