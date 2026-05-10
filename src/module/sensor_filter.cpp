// 超声波检测墙体类
#include "module/sensor_filter.h"

WallInfo SensorFilter::update(const SensorData& sensor)
{
    for(int i = 0;i<3;i++){
        float threshold = (i == 1) ? FRONT_DISTANCE_THRESHOLD : SIDE_DISTANCE_THRESHOLD;
        // 前墙：出现 5 帧确认，消失 10 帧确认（F 传感器噪声大，防止误触发）
        // 侧墙：出现 1 帧确认，消失 20 帧确认（路口拐角需要持续丢墙才承认）
        uint8_t appear_frames   = (i == 1) ? 5  : 1;
        uint8_t disappear_frames = (i == 1) ? 10 : 20;

        if (sensor.distance[i] < threshold) {
            below_count[i]++;
            above_count[i] = 0;
        } else {
            above_count[i]++;
            below_count[i] = 0;
        }

        // 确认有墙
        if (below_count[i] >= appear_frames) {
            if(i == 0) currentWallInfo.leftWall = true;
            else if(i == 1) currentWallInfo.frontWall = true;
            else if(i == 2) currentWallInfo.rightWall = true;
        }
        // 确认无墙
        else if (above_count[i] >= disappear_frames) {
            if(i == 0) currentWallInfo.leftWall = false;
            else if(i == 1) currentWallInfo.frontWall = false;
            else if(i == 2) currentWallInfo.rightWall = false;
        }
        // 过渡区：保持上一帧状态不变
    }
    return currentWallInfo;
}