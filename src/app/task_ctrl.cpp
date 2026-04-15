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
    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准

    bool isOver = false; // 是否完成迷宫，用于后续完善 send 功能
    SensorData sensorData;
    int yaw;
    osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever); // 获取初始航向角
    yaw = sensorData.Yaw;
    MotorCommand ctrl;
    uint8_t baseSpeed = 50; // 基础速度，后续完善 PID 时会用到
    uint8_t right_over_num = 0; // 右侧连续三次超出距离才右转，避免误判

    // 等待开始按钮按下
    for(;;){
        if(HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
            osDelay(20);
            if (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
                while(HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
                    osDelay(10);
                }
                break;
            }
        }
        osDelay(10);
    }

    for(;;){
        // 有限状态机
        switch (moveState)
        {
        case FORWARD:
            if(right_over_num >= 3){
                moveState = PRE_TURN;
                break;
            }
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            if(sensorData.distance[2] > 16){
                right_over_num++;
            } else {
                right_over_num = 0;
            }
            break;
        
        case PRE_TURN:
            /* code */
            break;

        case TURN:
            /* code */
            break;

        case AFTER_TURN:
            /* code */
            break;

        case STOP:
            ctrl.speed_percent[0] = 0;
            ctrl.speed_percent[1] = 0;
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            break;
        
        default:
            // 串口传报错
            break;
        }
        // 延时让出CPU
        osDelay(20);
    }
}
