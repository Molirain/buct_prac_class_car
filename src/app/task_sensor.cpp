#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/ultrasonic.h"
#include "driver/encoder.h"

void AppTaskSensor(void *argument) {
    // 1. 传感器初始化
    UltrasonicDriver_Init();

    // 2. 任务无限循环
    for(;;) {
        // 读取超声波、编码器、陀螺仪等数据...
        
        // 延时让出CPU
        osDelay(10);
    }
}
