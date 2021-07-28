


#include "utils/game_util.hpp"

typedef float valueType;
struct Rewards {
public:
    valueType CollReward = 1;
    valueType GoalReward = -0.9;
    valueType WallReward = -1;
    valueType Step_reward = 0.0;
    valueType discountF=0.987;//0.987;//0.987// 1.0;
    static Rewards getRewards()
    {
        return Rewards();
    }
};