#include "driver/encoder.h"

// 构造函数：告诉这个对象它归哪个定时器管
Encoder::Encoder(TIM_HandleTypeDef* timer) : htim(timer), lastCount(0) {
}

// 唤醒硬件解码器
void Encoder::begin() {
    // 开启定时器的编码器模式（双通道）
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    // 初始化计数值为 0
    __HAL_TIM_SET_COUNTER(htim, 0);
    lastCount = 0;
}

// 获取累计总脉冲数 (主要用于算总里程)
uint32_t Encoder::getTotalCount() {
    return __HAL_TIM_GET_COUNTER(htim);
}

// ★ 极其关键的方法：获取自上次调用以来的脉冲差值（用于计算当前速度）
int16_t Encoder::getDelta() {
    // 强制转为 16 位无符号数读取
    uint16_t currentCount = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    // 巧妙利用 C++ 的 16位溢出特性：
    // 如果电机反转，计数器从 0 变成 65535，(int16_t)(65535 - 0) 会自动变成 -1 ！
    int16_t delta = (int16_t)(currentCount - lastCount);
    // 更新历史记录
    lastCount = currentCount;
    return delta;
}

// 重置计数器
void Encoder::reset() {
    __HAL_TIM_SET_COUNTER(htim, 0);
    lastCount = 0;
}