#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "driver/ultrasonic.h"
#include "driver/encoder.h"
#include "driver/MPU6050.h"

extern I2C_HandleTypeDef hi2c1;
extern osMessageQueueId_t xSensorQueue;
MPU6050 gyro(&hi2c1);

void AppTaskSensor(void *argument) {
    gyro.begin();
    
    // 2. 任务无限循环
    for(;;) {
        // 等待 I2C DMA 的二值信号量，最大等待 10ms，非阻塞读取数据！
        gyro.update_DMA();

        osDelay(10);
    }
}
