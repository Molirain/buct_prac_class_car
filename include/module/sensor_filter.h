#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"

#define DISTANCE_THRESHOLD 20.0 // 距离阈值，单位 cm

class SensorFilter
{
    public:
        WallInfo update(const SensorData& sensorData);

    private:
        uint8_t over_num[3] = {0, 0, 0}; // 连续超过阈值的次数
        WallInfo currentWallInfo = {false, false, false}; // 当前的墙壁信息
};