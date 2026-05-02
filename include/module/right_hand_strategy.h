#pragma once

#include "maze_strategy.h"

class RightHandStrategy : public MazeStrategy {
public:
    RobotAction think(const WallInfo& walls, int x, int y, int heading) override;
    void reset() override;
};