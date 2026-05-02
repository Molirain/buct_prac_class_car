#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "main.h"
#include "driver/MPU6050.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void AppTaskMotor(void *argument);
void AppTaskSensor(void *argument);
void AppTaskCtrl(void *argument);
void AppTaskComm(void *argument);

#ifdef __cplusplus
}

struct MotorCommand
{
    double speed_percent[2]; // 速度百分比，-100 ~ 100
};

struct SensorData
{
    double distance[3]; // 0 ~ 2,左中右
    double Yaw;
    double speed[2]; // 左右轮速度，单位 m/s
    double L[2]; // 左右轮里程增量，单位 mm
};

#endif

#endif // APP_TASKS_H
