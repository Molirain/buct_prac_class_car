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

enum TURN_DIRECTION{
    LEFT,
    RIGHT,
    BACK
};

MOVE_STATE moveState = STOP;
TURN_DIRECTION nextTurn;

void AppTaskCtrl(void *argument) {
    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准

    // bool isOver = false; // 是否完成迷宫，用于后续完善 send 功能
    SensorData sensorData;
    SensorData lastSensorData;
    double yaw;
    osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever); // 获取初始航向角
    yaw = sensorData.Yaw;
    double target_yaw; // 目标角度
    MotorCommand ctrl;
    double baseSpeed = BASE_SPEED; // 基础速度，后续完善 PID 时会用到
    double right_dist_set = BASE_RIGHT_D; // 靠右的距离
    uint8_t right_over_num = 0; // 右侧连续三次超出距离才右转，避免误判
    uint8_t left_over_num = 0; // 左侧连续三次超出距离才左转，避免误判
    uint8_t back_over_num = 0; // 前方连续三次超出距离才后转，避免误判
    double last_turn_error;

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
                nextTurn = RIGHT;
                target_yaw = yaw + 90.0;
                last_turn_error = 90.0;
                break;
            }
            if(left_over_num >= 3){
                moveState = PRE_TURN;
                nextTurn = LEFT;
                target_yaw = yaw - 90.0;
                last_turn_error = -90.0;
                break;
            }
            if(back_over_num >= 3){
                moveState = PRE_TURN;
                nextTurn = BACK;
                target_yaw = yaw + 180.0;
                last_turn_error = 180.0;
                break;
            }
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            yaw = sensorData.Yaw;
            if(sensorData.distance[2] > 16){
                right_over_num++;
            } else {
                right_over_num = 0;
            }
            if(sensorData.distance[1] < 18){
                if(sensorData.distance[0] > 30){
                    left_over_num++;
                } else {
                    left_over_num = 0;
                    back_over_num++;
                }
            } else {
                left_over_num = 0;
                back_over_num = 0;
            }
            forward(&ctrl, baseSpeed, right_dist_set, &sensorData, &lastSensorData);
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            lastSensorData = sensorData;

            break;
        
        case PRE_TURN:
            /* code */
            break;

        case TURN:
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            yaw = sensorData.Yaw;

            if(turn(yaw, &last_turn_error, target_yaw, &ctrl)){
                moveState = AFTER_TURN;
                break;
            }
            last_turn_error = target_yaw - yaw;
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

void speedHold(MotorCommand* input)
{
    double left = input->speed_percent[0];
    double right = input->speed_percent[1];
    if(left < 0) left = -left;
    if(right < 0) right = -right;
    double max = left > right ? left : right;
    if(max > 100){
        input->speed_percent[0] = input->speed_percent[0] / max * 100;
        input->speed_percent[1] = input->speed_percent[1] / max * 100;
    }
}

void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData)
{
    double error = sensorData->distance[2] - right_distance_set;
    double last_error = lastSensorData->distance[2] - right_distance_set;
    double Kp = 2.0;
    double Kd = 1.0;

    ctrl->speed_percent[0] = baseSpeed + Kp * error + Kd * (error - last_error);
    ctrl->speed_percent[1] = baseSpeed - Kp * error - Kd * (error - last_error);
    speedHold(ctrl);
}

bool turn(double cur_yaw, double* last_error, double target_yaw, MotorCommand* ctrl)
{
    double error = target_yaw - cur_yaw;
    if(error < 2 && error > -2){
        return true;
    }
    // 这里简单的 P 控制，后续可以完善 PID

    return false;
}