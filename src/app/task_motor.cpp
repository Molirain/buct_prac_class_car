#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/motor.h"

void AppTaskMotor(void *argument) {
    // 1. 在这里做任务初始化
    // MotorDriver_Init(); // 已替换为C++类，可在外部实例化 Motor leftMotor(...) 并调用 leftMotor.begin()

    // 2. 任务无限循环
    for(;;) {
        // 电机控制PID计算、PWM更新等逻辑...
        
        // 延时让出CPU
        osDelay(10);
    }
}
