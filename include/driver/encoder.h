#pragma once
#include "stm32h7xx_hal.h"

class Encoder {
private:
    TIM_HandleTypeDef* htim;
    uint16_t lastCount;
public:
    // 构造函数：告诉这个对象它归哪个定时器管
    Encoder(TIM_HandleTypeDef* timer);
    
    // 唤醒硬件解码器
    void begin();
    
    // 获取累计总脉冲数 (主要用于算总里程)
    uint32_t getTotalCount();
    
    // ★ 极其关键的方法：获取自上次调用以来的脉冲差值（用于计算当前速度）
    int16_t getDelta();
    
    // 重置计数器
    void reset();
};