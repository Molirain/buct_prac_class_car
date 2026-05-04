#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/motor.h"

#define DEADZONE_L 14.5f // 死区
#define DEADZONE_R 16.0f // 死区

extern TIM_HandleTypeDef htim5;  // 左电机
extern TIM_HandleTypeDef htim12; // 右电机
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t xMotorQueue;

// 配平系数，针对左电机，右电机为基准，此处为前置反馈
float trim[102] = {
    0.00, 1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00,
    10.00, 11.00, 12.00, 13.00, 14.00, 14.44,
    14.44, 15.59, 16.82, 18.29, 19.12,
    20.01, 21.22, 22.02, 23.36, 24.38,
    25.17, 26.38, 26.89, 27.75, 28.09,
    28.97, 30.11, 30.65, 31.87, 33.05,
    34.03, 34.90, 36.13, 36.93, 38.05,
    38.77, 39.98, 40.83, 42.01, 42.95,
    43.65, 44.81, 46.16, 47.12, 48.73,
    48.81, 50.75, 53.63, 54.95, 55.39,
    57.03, 57.60, 58.00, 59.00, 60.00,
    61.00, 62.00, 63.00, 65.09, 67.18,
    68.01, 69.18, 71.13, 71.50, 72.50,
    74.13, 75.56, 75.33, 76.25, 76.95,
    78.57, 80.75, 79.09, 81.57, 82.48,
    83.50, 84.67, 86.35, 86.40, 86.57,
    86.00, 87.00, 88.00, 91.29, 90.00,
    91.39, 93.35, 93.00, 95.94, 96.19,
    97.01, 97.00, 98.54, 99.47, 100.00, 100.00
};

// 0为左，1为右
static Motor motor[2] = {
    Motor(&htim5, TIM_CHANNEL_3, TIM_CHANNEL_4, DEADZONE_L, trim, true), // 左
    Motor(&htim12, TIM_CHANNEL_2, TIM_CHANNEL_1, DEADZONE_R, nullptr, false)  // 右
};

void AppTaskMotor(void *argument) {
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
    }
}
