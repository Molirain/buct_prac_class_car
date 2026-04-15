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
};

// 保证速度在 -100 ~ 100 之间，按比例缩小
void speedHold(MotorCommand* input);

// 直行
void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData);

// 转弯
bool turn(double cur_yaw, double* last_error, double target_yaw, MotorCommand* ctrl);

// 直行，已距离为参数

#endif

#endif // APP_TASKS_H
