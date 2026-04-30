// 超声波检测墙体类
#include "module/sensor_filter.h"

WallInfo SensorFilter::update(const SensorData& sensor)
{
    for(int i = 0;i<3;i++){
        if(sensor.distance[i] < DISTANCE_THRESHOLD){
            over_num[i] = 0;
        } else {
            if(over_num[i] < 10) {
                over_num[i]++;
            }
        }
        if(over_num[i] >= 3){
            if(i == 0) currentWallInfo.leftWall = false;
            else if(i == 1) currentWallInfo.frontWall = false;
            else if(i == 2) currentWallInfo.rightWall = false;
        } else {
            if(i == 0) currentWallInfo.leftWall = true;
            else if(i == 1) currentWallInfo.frontWall = true;
            else if(i == 2) currentWallInfo.rightWall = true;
        }
    }
    return currentWallInfo;
}