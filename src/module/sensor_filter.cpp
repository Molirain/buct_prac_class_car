// 超声波检测墙体类
#include "module/sensor_filter.h"

WallInfo SensorFilter::update(const SensorData& sensor)
{
    for(int i = 0;i<3;i++){
        float threshold = (i == 1) ? FRONT_DISTANCE_THRESHOLD : SIDE_DISTANCE_THRESHOLD;
        // 前墙消失 5 帧(0.5s)确认；侧墙消失 20 帧(2.0s)确认。
        // 侧墙消失必须是真正持续没墙（路口），不能因为车身振荡导致几帧丢回波就误判。
        uint8_t disappear_frames = (i == 1) ? 5 : 20;

        // 迟滞滤波：有墙瞬间确认（1帧）；无墙持续 disappear_frames 帧才承认
        if(sensor.distance[i] < threshold){
            over_num[i] = 0;  // 瞬间跳到 0 → 立即承认"有墙"
        } else {
            if (over_num[i] < disappear_frames) over_num[i]++; 
        }

        // 达到上限才承认 "无墙"
        if(over_num[i] >= disappear_frames){ 
            if(i == 0) currentWallInfo.leftWall = false;
            else if(i == 1) currentWallInfo.frontWall = false;
            else if(i == 2) currentWallInfo.rightWall = false;
        } 
        // 只有降到底 0 才承认 "有墙"
        else if (over_num[i] == 0) {
            if(i == 0) currentWallInfo.leftWall = true;
            else if(i == 1) currentWallInfo.frontWall = true;
            else if(i == 2) currentWallInfo.rightWall = true;
        }
        // 中间状态 (0 < over_num < disappear_frames) 保持上一帧的历史记忆不变！
    }
    return currentWallInfo;
}