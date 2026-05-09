#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"

// 前墙判断阈值（停车距离），太大会导致还没到路口中心就停车，从而误判左右都有墙（掉头）
#define FRONT_DISTANCE_THRESHOLD 25.0 
// 侧墙判断阈值，需大于平常靠墙直行的距离
#define SIDE_DISTANCE_THRESHOLD 35.0 

class SensorFilter
{
    public:
        WallInfo update(const SensorData& sensorData);

    private:
        // 保守策略：初始默认"无墙"。有墙时1帧即可确认；无墙时必须连续N帧测不到才承认。
        // 这彻底避免了启动前静态积累导致的"迟迟认不出墙"问题。
        uint8_t over_num[3] = {0, 0, 0}; 
        WallInfo currentWallInfo = {false, false, false}; 
};