// 基类
#pragma once

#include "robot_types.h"

class MazeStrategy {
public:
    virtual ~MazeStrategy() = default;

    virtual RobotAction think(const WallInfo& walls, int x, int y, int heading) = 0; // walls为当前墙壁信息，x和y为当前逻辑坐标，heading为当前朝向，0北1东2南3西

    virtual void reset() = 0;
};