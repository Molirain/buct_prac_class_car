#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include <cstdio>
#include "robot_types.h"

extern UART_HandleTypeDef huart1;
extern SensorData sensorData; // task_sensor.cpp 已经声明了全局的 sensorData
extern WallInfo g_walls;       // task_ctrl.cpp 计算后的墙壁信息
extern RobotAction g_lastAction;

void AppTaskComm(void *argument) {
    // 1. 通信外设初始化 (OLED, 串口等)
    char buf[128];
    
    // 2. 任务无限循环
    for(;;) {
        // 打印三个方向的原始距离和滤波后的墙壁状态 (1有墙，0无墙)，以及大脑最后一次做出的决断
        int len = snprintf(buf, sizeof(buf), "L:%05.1f F:%05.1f R:%05.1f | WL:%d WF:%d WR:%d | ACT:%d\r\n", 
                           sensorData.distance[0], 
                           sensorData.distance[1], 
                           sensorData.distance[2],
                           g_walls.leftWall,
                           g_walls.frontWall,
                           g_walls.rightWall,
                           (int)g_lastAction);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
        
        // 延时100ms（10Hz的打印频率足够调试，不会卡死）
        osDelay(100);
    }
}
