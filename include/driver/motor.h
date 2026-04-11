#pragma once
#include "stm32h7xx_hal.h"

class Motor {
private:
    TIM_HandleTypeDef* htim;
    uint32_t ch_in1;
    uint32_t ch_in2;
public:
    Motor(TIM_HandleTypeDef* timer, uint32_t in1, uint32_t in2);
    
    void begin(); 
    
    // 调速方法：传入 -100 到 100 的速度值（正数前进，负数后退）
    void setSpeed(int speed);
};