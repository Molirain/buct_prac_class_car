#pragma once
#include "stm32h7xx_hal.h"

class Motor {
private:
    TIM_HandleTypeDef* htim;
    uint32_t ch_in1;
    uint32_t ch_in2;
    float deadband; // 死区
    float* trim; // 配平系数数组
    bool should_trim; // 是否启用配平功能
    double to_trim(double speed); // 根据配平系数调整速度

public:
    Motor(TIM_HandleTypeDef* timer, uint32_t in1, uint32_t in2, float deadband = 15.0f, float* trim = nullptr, bool should_trim = false);
    void begin(); 
    
    // 调速方法：传入 -100 到 100 的速度值（正数前进，负数后退）
    void setSpeed(double speed);
};