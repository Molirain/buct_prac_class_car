#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/motor.h"

#define DEADZONE 20.0f // 死区

extern TIM_HandleTypeDef htim5;  // 左电机
extern TIM_HandleTypeDef htim12; // 右电机
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t xMotorQueue;

// 配平系数，针对左电机，右电机为基准，此处为前置反馈
float trim[102] = {0.0f, 5.0f, 9.5f, 13.5f, 17.0f, 20.0f, 22.5f, 25.0f, 27.0f, 29.0f, 
                    30.5f, 32.0f, 33.5f, 35.0f, 36.0f, 37.0f, 38.0f, 39.0f, 40.0f, 41.0f,
                    42.0f, 43.0f, 44.0f, 45.0f, 46.0f, 47.0f, 48.0f, 49.0f, 50.0f, 
                    51.0f, 52.0f, 53.0f, 54.0f, 55.0f, 56.0f, 57.0f, 58.0f, 59.0f,
                    60.0f, 61.0f, 62.0f, 63.0f, 64.0f, 65.0f, 66.5f, 68.5f, 
                    70.5f, 72.5f, 75.5f, 80.5f,85.5f ,90.5f ,95.5f ,100.f, 100.f};

void AppTaskMotor(void *argument) {
    // 0为左，1为右
    Motor motor[2] = {
        Motor(&htim5, TIM_CHANNEL_3, TIM_CHANNEL_4, DEADZONE, trim, true), // 左
        Motor(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, DEADZONE, nullptr, false)  // 右
    };
    motor[0].begin();
    motor[1].begin();
    
    MotorCommand cmd;
    cmd.speed_percent[0] = 0;
    cmd.speed_percent[1] = 0;
    motor[0].setSpeed(0);
    motor[1].setSpeed(0);

    HAL_UART_Transmit(&huart1, (uint8_t*)"TASK Motor Start!\r\n", 19, 100);

    for(;;){
        if(osMessageQueueGet(xMotorQueue, &cmd, NULL, osWaitForever) == osOK) {
            // 收到新命令才更新电机速度
            motor[0].setSpeed(cmd.speed_percent[0]);
            motor[1].setSpeed(cmd.speed_percent[1]);
        }
        osDelay(10);
    }
}
