#include <iostream>
#include "src/fileIO/csv_util.hpp"
#include "src/utils/game_util.hpp"
#include "src/fileIO/processerCSV.hpp"
#include "States/State.hpp"
#include "States/Data/StateData.hpp"
#include "Game/Initializer.hpp"
#include "Game/Sim.hpp"
#include "Agnets/PathDecomposition/PathDecomposit.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

void game_start(configGame &conf);
int get_random_time();
int main() {

    int seed=get_random_time();
    cout<<get_random_time()<<endl;
    std::cout << "Hello, World!" << std::endl;
    std::string csv_path= getProjectDir() + "/csv/con2.csv";
    cout<<"csv_path: "<<csv_path<<endl;
    CSVReader reader(csv_path,',');
    vector<vector<string>> rowsCsv = reader.getDataCSV();
    auto root_dir = getRootDir();
    // rm all files in log_dir
    OS::deleteDirectoryContents(root_dir+LOG_DIR);
    //log the con.csv file
    Logger::copy_file(root_dir+LOG_DIR,csv_path);
    Logger loggerTime(root_dir,"time");
    std::vector<int> gr_id = {};
    bool is_saved= true;
    for (int i = 1; i < rowsCsv.size(); ++i) {
        seed = get_random_time();
        auto conf = configGame(rowsCsv[i],seed);
        srand(conf._seed);
        //conf.maxA=3;
        // all the mode with -1 are GR naive agents
        if (conf.mode<0) {gr_id.push_back(i);continue;}
        if (conf.mode==1 and conf.h==3) conf.h=2;

        auto grid = Initializer::init_grid(conf.sizeGrid,conf.gGoals,conf.probGoals);
        State s;
        s.g_grid=grid.get();
        auto adata = EXtraData();
        s.add_player_state(agentEnum::A,conf.posAttacker.front(),Point(0),adata);
        s.add_player_state(agentEnum::D,conf.posDefender.front(),Point(0),adata);
        cout<<s.to_string_state()<<endl;


        auto evder_agent = Initializer::init_attacker(conf, is_saved );
        auto pursurer_agent = Initializer::init_RTDP(conf,evder_agent.get());

        is_saved=false;
        PathDecomposit dec = PathDecomposit(evder_agent.get(),conf,s);

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        dec.train();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;

        loggerTime.log_string_row({conf.idNumber,std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())});
    }

    for (int idx : gr_id) {
        auto conf = configGame(rowsCsv[idx],seed);
        game_start(conf);
    }

    return 0;
}

void game_start(configGame &conf)
{
    srand(conf._seed);
    auto grid = Initializer::init_grid(conf.sizeGrid,conf.gGoals,conf.probGoals);
    State s;
    s.g_grid=grid.get();
    auto adata = EXtraData();
    s.add_player_state(agentEnum::A,conf.posAttacker.front(),Point(0),adata);
    s.add_player_state(agentEnum::D,conf.posDefender.front(),Point(0),adata);
    cout<<s.to_string_state()<<endl;

    auto evder_agent = Initializer::init_attacker(conf, false);
    auto pursurer_agent = Initializer::init_GR(conf,evder_agent.get());
    auto sim  = Emulator(pursurer_agent.get(),evder_agent.get(), std::move(s),conf);
    sim.main_loop(8000);//2000000
}

int get_random_time()
{
    int seed_me=0;
    auto start = std::chrono::system_clock::now();
    // Some computation here
    std::this_thread::sleep_for(1ms);

    auto end = std::chrono::system_clock::now();

    auto s1 = std::chrono::duration_cast<std::chrono::nanoseconds> (end-start).count();

    seed_me = s1;
    return seed_me+time(nullptr)%1000;
}