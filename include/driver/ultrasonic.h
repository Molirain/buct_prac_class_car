#pragma once

#include "main.h"

#define MAX_SONARS 3

class SonarSensor {
private:
    GPIO_TypeDef* triggerPort;
    uint16_t triggerPin;
    TIM_HandleTypeDef* htimIC;
    uint32_t timChannel;

    volatile uint32_t risingTime;
    volatile uint32_t fallingTime;
    volatile bool isWaitingEcho;
    uint32_t triggerStartTick;
    
    // 用来记录当前是在等上升沿还是下降沿，替代读寄存器
    volatile bool isWaitingFallingEdge; 

    volatile float distanceCm;
    const float MAX_DIST = 200.0f;
    static constexpr uint32_t ECHO_TIMEOUT_MS = 30;

    static SonarSensor* instanceMap[MAX_SONARS];

public:
    SonarSensor(GPIO_TypeDef* trigPort, uint16_t trigPin, 
                TIM_HandleTypeDef* htim, uint32_t channel);

    void Init();
    void Trigger();
    float GetDistanceCm();
    bool IsBusy() { return isWaitingEcho; }

    // 参数精简，只需要传入捕获值
    void Internal_IC_Callback(uint32_t capture_value);

    // 静态路由函数，供外部的 C 中断回调使用
    static void TIM_IC_Callback_Router(TIM_HandleTypeDef *htim);
};