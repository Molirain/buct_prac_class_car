#pragma once
#include "stm32h7xx_hal.h"

class Motor {
private:
    TIM_HandleTypeDef* htim;
    uint32_t ch_in1;
    uint32_t ch_in2;
public:
    // 构造函数：告诉这个对象它归哪个定时器管
    Motor(TIM_HandleTypeDef* timer, uint32_t in1, uint32_t in2);
    
    // 初始化：启动硬件 PWM
    void begin();
    
    // 终极调速方法：传入 -100 到 100 的速度值（正数前进，负数后退）
    void setSpeed(int speed);
};