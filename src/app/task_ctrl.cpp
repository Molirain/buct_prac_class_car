#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t xMotorQueue;
extern osMessageQueueId_t xSensorQueue;

enum MOVE_STATE{
    FORWARD,
    PRE_TURN,
    TURN,
    AFTER_TURN,
    STOP
};

MOVE_STATE moveState = STOP;

void AppTaskCtrl(void *argument) {
    // 1. 算法初始化
    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准

    SensorData sensorData;
    int yaw;
    osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever); // 获取初始航向角
    yaw = sensorData.Yaw;

    // 2. 任务无限循环
    for(;;) {
        // 迷宫算法求解 / 路径规划...
        
        // 延时让出CPU
        osDelay(20);
    }
}
