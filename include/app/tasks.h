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

// 保证速度在 -100 ~ 100 之间，按比例缩小
void speedHold(MotorCommand* input);

// 直行
void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData);

// 靠编码器速度差控制直行
void forward_withDiff(MotorCommand* ctrl, double baseSpeed, SensorData* sensorData, SensorData* lastSensorData);

// 转弯
bool turn(double target_angle, double* accum_L, double* last_error_L, MotorCommand* ctrl);

// 直行，已距离为参数

#endif

#endif // APP_TASKS_H
