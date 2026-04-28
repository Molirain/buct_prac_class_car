#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/motor.h"

#define DEADZONE 20.0f // 死区

extern TIM_HandleTypeDef htim5;  // 左电机
extern TIM_HandleTypeDef htim12; // 右电机
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t xMotorQueue;

void AppTaskMotor(void *argument) {
    // 0为左，1为右
    Motor motor[2] = {
        Motor(&htim5, TIM_CHANNEL_3, TIM_CHANNEL_4, DEADZONE, 1.0f), // 左
        Motor(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, DEADZONE, 1.0f)  // 右
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
