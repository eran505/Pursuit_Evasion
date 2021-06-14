cmake_minimum_required(VERSION 3.17)
project(PE)

set(CMAKE_CXX_STANDARD 20)
include_directories(src)
#include_directories(Tests)

#set(CMAKE_CXX_FLAGS_DEFAULT "-O3 -g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#set(CMAKE_CXX_FLAGS_DEBUG  "-O3 -g")

#set(CMAKE_CXX_FLAGS -DDEBUG)
#set(CMAKE_CXX_FLAGS "-Wall " )

#add_subdirectory(lib/googletest-master)
#include_directories(lib/googletest-master/googletest/include)

#include_directories(lib/googletest-master/googlemock/include)

add_executable(PE main.cpp src/fileIO/csv_util.hpp src/fileIO/saver.hpp src/fileIO/processerCSV.hpp src/utils/game_util.hpp src/utils/game_util.cpp src/utils/printFunc.hpp src/Game/Grid.hpp src/States/State.hpp src/States/State.cpp src/Game/Initializer.hpp src/States/Data/StateData.hpp src/Search/Astar.hpp src/Search/Astar.cpp src/Attacker/ABpathFinder.hpp src/utils/Rand.hpp src/Attacker/Normalizer.hpp src/Attacker/PathGenrator.hpp src/Attacker/StaticPolicy.hpp src/Attacker/PathMapper.hpp src/Agnets/Policer.hpp src/GoalRec/PathRec.hpp src/GoalRec/AgentGR.hpp src/Game/Simulator.hpp src/Game/Sim.hpp src/utils/logeer.h src/Agnets/DP/RTDP.h src/Agnets/DP/ActionExpender.h src/Agnets/DP/RtdpUtil.h src/Agnets/DP/MemoryRTDP.hpp src/Agnets/DP/Rewards.hpp src/Agnets/TrajectoriesTree.hpp src/Agnets/DP/EvaluatorActionzer.hpp src/GoalRec/NodeGR.hpp)

#target_link_libraries(PE gtest gtest_main)
