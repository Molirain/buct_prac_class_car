#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

void AppTaskComm(void *argument) {
    // 1. 通信外设初始化 (OLED, 串口等)
    
    // 2. 任务无限循环
    for(;;) {
        // 处理串口收发、刷新OLED屏幕...
        
        // 延时让出CPU
        osDelay(50);
    }
}
