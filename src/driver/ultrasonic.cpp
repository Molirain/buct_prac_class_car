#include "driver/ultrasonic.h"

SonarSensor* SonarSensor::instanceMap[MAX_SONARS] = {nullptr};

static inline void DelayUs(uint32_t us) {
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

SonarSensor::SonarSensor(GPIO_TypeDef* trigPort, uint16_t trigPin, 
                         TIM_HandleTypeDef* htim, uint32_t channel) 
    : triggerPort(trigPort), triggerPin(trigPin), htimIC(htim), timChannel(channel) {
    
    risingTime = 0;
    fallingTime = 0;
    lastTriggerTime = 0;
    distanceCm = MAX_DIST;
    isWaitingEcho = false;
    isWaitingFallingEdge = false; // 初始化为不等待下降沿

    uint8_t index = (channel >> 2) & 0x03;
    if (index < MAX_SONARS) {
        instanceMap[index] = this;
    }
}

void SonarSensor::Init() {
    __HAL_TIM_SET_CAPTUREPOLARITY(htimIC, timChannel, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(htimIC, timChannel);
}

void SonarSensor::Trigger() {
    if (isWaitingEcho) {
        // HC-SR04 最长回波大约在 38ms 结束，如果超过 50ms 还没触发回调，说明丢中断或线异常了，触发超时自恢复
        if (HAL_GetTick() - lastTriggerTime < 50) {
            return; // 还在合理通信时间内，坚决不打断原流程
        }
        // 以下是超时急救：清空假死状态
        distanceCm = MAX_DIST; 
    }

    lastTriggerTime = HAL_GetTick();
    isWaitingEcho = true;
    isWaitingFallingEdge = false; // 触发时，复位状态，准备先抓上升沿

    // 万一卡在硬件极性等下降沿阶段，强制扭回抓上升沿（遵循 ST 官方的安全顺序）
    TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_DISABLE);
    __HAL_TIM_SET_CAPTUREPOLARITY(htimIC, timChannel, TIM_INPUTCHANNELPOLARITY_RISING);
    TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_ENABLE);

    HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);
    DelayUs(12); // HC-SR04 needs >=10us trigger high pulse
    HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
}

float SonarSensor::GetDistanceCm() {
    return distanceCm;
}

// --- 核心逻辑修改：用状态位判断极性 ---
void SonarSensor::Internal_IC_Callback(uint32_t capture_value) {
    if (!isWaitingEcho) {
        return;
    }

    if (!isWaitingFallingEdge) {
        // 状态1：当前没有在等下降沿，说明抓到的是上升沿
        risingTime = capture_value;
        isWaitingFallingEdge = true; // 切换状态：下次就是抓下降沿了
        
        // 硬件极性切为下降沿（必须先Disable通道再修改极性才最安全）
        TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_DISABLE);
        __HAL_TIM_SET_CAPTUREPOLARITY(htimIC, timChannel, TIM_INPUTCHANNELPOLARITY_FALLING);
        TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_ENABLE);
    } 
    else {
        // 状态2：抓到的是下降沿
        fallingTime = capture_value;
        isWaitingFallingEdge = false; // 测距完整结束，复位状态
        
        uint32_t pulseWidthUs;
        if (fallingTime >= risingTime) {
            pulseWidthUs = fallingTime - risingTime;
        } else {
            pulseWidthUs = (0xFFFF - risingTime) + fallingTime + 1;
        }

        distanceCm = (pulseWidthUs * 0.034f) / 2.0f;
        if (distanceCm > MAX_DIST) {
            distanceCm = MAX_DIST;
        }

        // 硬件极性切回上升沿（必须先Disable通道再修改极性才最安全），准备下一次触发
        TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_DISABLE);
        __HAL_TIM_SET_CAPTUREPOLARITY(htimIC, timChannel, TIM_INPUTCHANNELPOLARITY_RISING);
        TIM_CCxChannelCmd(htimIC->Instance, timChannel, TIM_CCx_ENABLE);
        isWaitingEcho = false; 
    }
}

// --- 【新增】C++ 静态路由函数 ---
void SonarSensor::TIM_IC_Callback_Router(TIM_HandleTypeDef *htim) {
    HAL_TIM_ActiveChannel activeChannel = HAL_TIM_GetActiveChannel(htim);
    uint8_t index = 255;

    switch (activeChannel) {
        case HAL_TIM_ACTIVE_CHANNEL_1: index = 0; break;
        case HAL_TIM_ACTIVE_CHANNEL_2: index = 1; break;
        case HAL_TIM_ACTIVE_CHANNEL_3: index = 2; break;
        default: return; 
    }

    // 这里是类的内部，可以合法访问 private 成员 instanceMap 和 timChannel
    SonarSensor* obj = instanceMap[index];

    if ((obj != nullptr) && (obj->htimIC == htim)) {
        uint32_t capVal = HAL_TIM_ReadCapturedValue(htim, obj->timChannel);
        obj->Internal_IC_Callback(capVal);
    }
}

// ==============================================================================
// === extern "C" 区域精简 ===
// ==============================================================================
extern "C" {

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // 纯 C 中断函数现在非常干净，直接把指针扔给 C++ 的静态路由去处理
    SonarSensor::TIM_IC_Callback_Router(htim);
}

}
