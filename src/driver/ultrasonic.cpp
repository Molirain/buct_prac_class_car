#include "driver/ultrasonic.h"

// 超声波初始化
void UltrasonicDriver_Init(void) {
    // 配置定时器输入捕获，启动超声波等...
}

// 读取前方超声波距离
float UltrasonicDriver_ReadFront(void) {
    // 触发 TRIG_FRONT_Pin 并读取回响
    return 0.0f;
}

// 读取左侧超声波距离
float UltrasonicDriver_ReadLeft(void) {
    return 0.0f;
}

// 读取右侧超声波距离
float UltrasonicDriver_ReadRight(void) {
    return 0.0f;
}
