#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "main.h"
#include "driver/MPU6050.h"

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
    int8_t speed_percent[2]; // 速度百分比，-100 ~ 100
};

struct SensorData
{
    int16_t distance[3]; // 0 ~ 2,左中右
    double Yaw;
};
#endif

#endif // APP_TASKS_H
