#include "driver/motor.h"
#include <cmath>

Motor::Motor(TIM_HandleTypeDef* timer, uint32_t in1, uint32_t in2) 
    : htim(timer), ch_in1(in1), ch_in2(in2) {
    // 使用初始化列表进行成员初始化
}

void Motor::begin() {
    HAL_TIM_PWM_Start(htim, ch_in1);
    HAL_TIM_PWM_Start(htim, ch_in2);
}

void Motor::setSpeed(int speed) {
    // 1. 限幅保护，防止输入越界
    if(speed > 100) speed = 100;
    if(speed < -100) speed = -100;
    
    // 2. 动态获取当前定时器的 ARR 值
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    // 防止 uint32_t 溢出，使用 uint64_t 计算
    uint32_t pwm_value = (uint32_t)(((uint64_t)std::abs(speed) * arr) / 100);
    
    // 3. 硬件下发执行
    if (speed > 0) {
        // 正转：IN1 脉冲，IN2 拉低
        __HAL_TIM_SET_COMPARE(htim, ch_in1, pwm_value);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, 0);
    }
    else if (speed < 0) {
        // 反转：IN1 拉低，IN2 脉冲
        __HAL_TIM_SET_COMPARE(htim, ch_in1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, pwm_value);
    }
    else {
        // 刹车：全拉高或全拉低 (优先使用全 0，即双低coast，适应更多驱动板防直通并解决起步乱转)
        __HAL_TIM_SET_COMPARE(htim, ch_in1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, 0);
    }
}