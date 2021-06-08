#include <iostream>
#include "src/fileIO/csv_util.hpp"
#include "src/utils/game_util.hpp"
#include "src/fileIO/processerCSV.hpp"
#include "utils/printFunc.hpp"
#include "States/State.hpp"
#include "States/Data/StateData.hpp"
#include "Game/Initializer.hpp"
#include "Attacker/StaticPolicy.hpp"
int main() {
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
    auto adata = Single(3);
    s.add_player_state(agentEnum::A,conf.posAttacker.front(),zero,adata);
    s.add_player_state(agentEnum::D,conf.posDefender.front(),Point(0),Single(10));
    cout<<s.to_string_state()<<endl;
    doPrint(cout,"hash:=",s.getHashValue());

    auto attacker = Initializer::init_attacker(conf);


    return 0;
}
