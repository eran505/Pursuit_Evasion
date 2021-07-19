//
// Created by eranhe on 6/12/21.
//

#ifndef PE_LOGEER_H
#define PE_LOGEER_H
#include <utility>
#include <filesystem>

#include "utils/game_util.hpp"
#include "fileIO/saver.hpp"
#include "fileIO/processerCSV.hpp"

namespace fs = std::filesystem;
int writeFile (const std::string& input,const string& home);
int writeFile (const std::string& input,const string& home)
{
    std::ofstream out(home+LOG_DIR"time.txt");
    out << input;
    out.close();
    return 0;
}

class Logger{
    std::vector<u_int32_t> info;
    //std::vector<u_int32_t> history;
    u_int64_t counter=0;
    std::string home;
    Saver<string> file_manger;
    u_int32_t log_every=1000;
    bool done = false;
public:
    enum info_val {ITER,COLL,GOAL,WALL};
    explicit Logger(const configGame& conf):info(4),home(conf.home),
    file_manger(conf.home+LOG_DIR+"S"+std::to_string(conf._seed)+"_I"+conf.idNumber+"_M"+std::to_string(conf.mode)+"_O"+std::to_string(conf.options)+"_H"+std::to_string(conf.h)+"_A"+std::to_string(conf.a)+"_Eval.csv",10)
    {
        file_manger.set_header_vec({"episodes","Collision","Wall" ,"Goal"});
    }
    explicit Logger(const string& home,const string& file_name):info(4),home(home),
                                            file_manger(home+LOG_DIR+"S"+file_name+".csv",10)
    {
        file_manger.set_header_vec({"ID","Time"});
    }
    bool get_done()const{return done;};
    bool set_done(bool bol){done=bol;};
public:
    void log_scalar_increment(info_val val){
        info[val]++;
        info[ITER]++;
        if (info[ITER]%log_every==0){
            counter++;
            print();
            flush();
        }
    }
    void log_string_row(vector<string>&& data_row)
    {
        file_manger.inset_data(data_row);
        file_manger.inset_endLine();
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

    static void copy_file(const string &dest,const string &src)
    {
        fs::path sourceFile = src;
        fs::path targetParent = dest;
        auto target = targetParent / sourceFile.filename();

        try // If you want to avoid exception handling, then use the error code overload of the following functions.
        {
            fs::create_directories(targetParent); // Recursively create target directory if not existing.
            fs::copy_file(sourceFile, target, fs::copy_options::overwrite_existing);
        }
        catch (std::exception& e) // Not using fs::filesystem_error since std::bad_alloc can throw too.
        {
            std::cout << e.what();
        }
    }

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
