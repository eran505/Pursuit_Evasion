


#include "utils/game_util.hpp"

typedef float valueType;
struct Rewards {
public:
    valueType CollReward = 1;
    valueType GoalReward = -0.9;
    valueType WallReward = -1;
    valueType Step_reward = 0.0;
    valueType discountF=0.988;//0.987;
    static Rewards getRewards()
    {
        return Rewards();
    }
};