#include "app/tasks.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include <queue.h>
#include "driver/ultrasonic.h"
#include "driver/encoder.h"
#include "driver/MPU6050.h"
#include "driver/DiffDriveOdometry.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern osMessageQueueId_t xSensorQueue;
extern UART_HandleTypeDef huart1;

static const EncoderConfig kEncoderCfg = {
    .wheelDiameter_m = 0.048f,   // 48mm 轮径（根据实物修改）
    .gearRatio       = 20.0f,    // 实际减速比 1:20
    .encoderLines    = 13,       // 霍尔13线；GMR编码器改为500
    .decodeMultiplier = 2.0f,    // 按实测标定：当前硬件计数等效 2 倍频
    .dt_s            = 0.01f,    // 10ms 控制周期
    .alpha           = 0.3f,     // IIR滤波系数 (0,1]，越小越平滑
};

MPU6050 gyro(&hi2c1);
// 模板参数与定时器位宽严格对应
// TIM2 = 32-bit, TIM3 = 16-bit
static Encoder<uint32_t> gEncLeft (&htim2, kEncoderCfg);   // 左轮编码器
static Encoder<uint16_t> gEncRight(&htim3, kEncoderCfg);   // 右轮编码器

extern TIM_HandleTypeDef htim4;
static SonarSensor sonarLeft(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, &htim4, TIM_CHANNEL_2);
static SonarSensor sonarFront(TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, &htim4, TIM_CHANNEL_1);
static SonarSensor sonarRight(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, &htim4, TIM_CHANNEL_3);

// 里程计：轴距145mm
static DiffDriveOdometry classic(0.145f);
SensorData sensorData;

void AppTaskSensor(void *argument) {
    QueueHandle_t xSQ = (QueueHandle_t)xSensorQueue;
    gyro.begin();
    gEncLeft.begin();
    gEncRight.begin();
    sonarLeft.Init();
    sonarFront.Init();
    sonarRight.Init();

    // 冷启动时先做几轮预读，丢弃首帧偶发异常值（常见为 200cm）。
    for (int i = 0; i < 3; i++) {
        sonarLeft.Trigger();
        sonarFront.Trigger();
        sonarRight.Trigger();
        osDelay(pdMS_TO_TICKS(25));
    }

    sensorData.distance[0] = sonarLeft.GetDistanceCm();
    sensorData.distance[1] = sonarFront.GetDistanceCm();
    sensorData.distance[2] = sonarRight.GetDistanceCm();

    HAL_UART_Transmit(&huart1, (uint8_t*)"TASK Sensor Start!\r\n", 20, 100);
    uint32_t tick = osKernelGetTickCount();

    for(;;) {
        // 阻塞读取 MPU6050 陀螺仪 Z 轴角速度，积分计算航向角
        gyro.update();
        sensorData.Yaw = gyro.getYaw();

        sonarLeft.Trigger();
        sonarFront.Trigger();
        sonarRight.Trigger();

        float vLeft  = -gEncLeft.update();   // m/s，左轮方向取反
        float vRight = gEncRight.update();   // m/s，已滤波

        // 计算位移增量（用于里程计）
        constexpr float kDt = 0.01f;
        float dLeft  = vLeft  * kDt;
        float dRight = vRight * kDt;

        // 更新里程计
        classic.update(dLeft, dRight);

        int32_t l_mm = (int32_t)(gEncLeft.getOdometryDistance_m() * 1000.0f);
        int32_t r_mm = (int32_t)(gEncRight.getOdometryDistance_m() * 1000.0f);

        sensorData.distance[0] = sonarLeft.GetDistanceCm();
        sensorData.distance[1] = sonarFront.GetDistanceCm();
        sensorData.distance[2] = sonarRight.GetDistanceCm();

        sensorData.speed[0] = vLeft;
        sensorData.speed[1] = vRight;
        sensorData.L[0] = l_mm;
        sensorData.L[1] = r_mm;

        xQueueOverwrite(xSQ, &sensorData);

        tick += pdMS_TO_TICKS(10);
        osDelayUntil(tick);
    }
}
