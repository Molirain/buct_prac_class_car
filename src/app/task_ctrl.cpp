#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

void AppTaskCtrl(void *argument) {
    // 1. 算法初始化
    
    // 2. 任务无限循环
    for(;;) {
        // 迷宫算法求解 / 路径规划...
        
        // 延时让出CPU
        osDelay(20);
    }
}
