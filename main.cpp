#include <iostream>
#include "src/fileIO/csv_util.hpp"
#include "src/utils/game_util.hpp"
#include "src/fileIO/processerCSV.hpp"
#include "States/State.hpp"
#include "States/Data/StateData.hpp"
#include "Game/Initializer.hpp"
#include "Game/Sim.hpp"

int main() {

    int seed=3;
    auto zero = Point(0);
    std::cout << "Hello, World!" << std::endl;
    std::string csv_path= getProjectDir() + "/csv/con16.csv";
    cout<<"csv_path: "<<csv_path<<endl;
    CSVReader reader(csv_path,',');
    vector<vector<string>> rowsCsv = reader.getDataCSV();



    for (int i = 1; i < rowsCsv.size(); ++i) {
        srand(seed);
        auto conf = configGame(rowsCsv[i],seed);

        auto grid = Initializer::init_grid(conf.sizeGrid,conf.gGoals,conf.probGoals);
        State s;
        s.g_grid=grid.get();
        auto adata = Complex();
        s.add_player_state(agentEnum::A,conf.posAttacker.front(),zero,adata);
        s.add_player_state(agentEnum::D,conf.posDefender.front(),Point(0),adata);
        cout<<s.to_string_state()<<endl;

        auto evder_agent = Initializer::init_attacker(conf);
        //auto pursurer_agent = Initializer::init_GR(conf,evder_agent.get());
        auto pursurer_agent = Initializer::init_RTDP(conf,evder_agent.get());
        auto sim  = Emulator(pursurer_agent.get(),evder_agent.get(), std::move(s),conf);
        sim.main_loop(200000);//2000000
        break;
    }



    return 0;
}
