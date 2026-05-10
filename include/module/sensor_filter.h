#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"

// 前墙判断阈值（停车距离），太大会导致还没到路口中心就停车，从而误判左右都有墙（掉头）
#define FRONT_DISTANCE_THRESHOLD 30.0 
// 侧墙判断阈值，需大于平常靠墙直行的距离
#define SIDE_DISTANCE_THRESHOLD 50.0 

class SensorFilter
{
    public:
        WallInfo update(const SensorData& sensorData);

    private:
        // 每传感器独立跟踪连续帧数：
        // below_count[i]：连续低于阈值的帧数 → 达标 → 确认"有墙"
        // above_count[i]：连续高于阈值的帧数 → 达标 → 确认"无墙"
        uint8_t below_count[3] = {0, 0, 0};
        uint8_t above_count[3] = {0, 0, 0};
        WallInfo currentWallInfo = {false, false, false}; 
};