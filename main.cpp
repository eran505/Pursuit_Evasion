#include <iostream>
#include "src/fileIO/csv_util.hpp"
#include "src/utils/game_util.hpp"
#include "src/fileIO/processerCSV.hpp"
#include "utils/printFunc.hpp"
#include "States/State.hpp"
#include "States/Data/StateData.hpp"
#include "Game/Initializer.hpp"
#include "Attacker/StaticPolicy.hpp"
//#include "Tests/test_exp.hpp"
//#include "gtest/gtest.h"
#include "Game/Sim.hpp"

int main(int argc, char **argv) {

//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();

    auto zero = Point(0);
    std::cout << "Hello, World!" << std::endl;
    std::string csv_path= getProjectDir() + "/csv/con16.csv";
    cout<<"csv_path: "<<csv_path<<endl;
    CSVReader reader(csv_path,',');
    vector<vector<string>> rowsCsv = reader.getDataCSV();
    auto conf = configGame(rowsCsv[1],3);
    auto init  = Initializer();


    cout<<rowsCsv[0]<<endl;
    auto res = to_csv2(rowsCsv[0]);
    cout<<res<<endl;
    auto grid = Initializer::init_grid(conf.sizeGrid,conf.gGoals,conf.probGoals);
    State s;
    s.g_grid=grid.get();
    auto adata = Single(1);
    s.add_player_state(agentEnum::A,conf.posAttacker.front(),zero,adata);
    s.add_player_state(agentEnum::D,conf.posDefender.front(),Point(0),adata);
    cout<<s.to_string_state()<<endl;
    doPrint(cout,"hash:=",s.getHashValue());
    auto evder_agent = Initializer::init_attacker(conf);
    auto pursurer_agent = Initializer::init_defender(conf,evder_agent.get());
    auto sim  = Emulator(pursurer_agent.get(),evder_agent.get(), std::move(s));
    sim.main_loop(1000000);

    return 0;
}
