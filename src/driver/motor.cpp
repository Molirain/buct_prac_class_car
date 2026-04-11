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
    
    // 2. 将百分比转换为 PWM 比较值 (假设你在 CubeMX 里把 Counter Period(ARR) 设为了 1000)
    // 如果你的 ARR 是其他值，请把这里的 1000 换成你的 ARR 值！
    uint32_t pwm_value = (std::abs(speed) * 1000) / 100;
    
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
        // 刹车：全拉高或全拉低
        __HAL_TIM_SET_COMPARE(htim, ch_in1, 1000);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, 1000);
    }
}