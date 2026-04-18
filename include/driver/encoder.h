#pragma once
/**
 * @file    Encoder.h
 * @brief   单轮编码器驱动（模板类头文件）
 *
 * 职责边界（单一职责原则）：
 *   ✅ 本类负责：脉冲采样、物理单位换算（脉冲→m/s）、IIR 低通滤波、
 *               【绝对里程】累积（标量，永远为正，倒车也累加）
 *   ❌ 本类不负责：X/Y 坐标、航向角 θ ——那是 DiffDriveOdometry 的职责
 *
 * 模板参数 CounterType：
 *   uint16_t → 16-bit 定时器（TIM3 等）
 *   uint32_t → 32-bit 定时器（TIM2 等）
 *
 * 使用示例：
 *   Encoder<uint32_t> encLeft (&htim2, cfg);  // TIM2, 32-bit
 *   Encoder<uint16_t> encRight(&htim3, cfg);  // TIM3, 16-bit
 *   encLeft.begin();
 *   // 在 10ms 控制任务中：
 *   float vLeft = encLeft.update();   // m/s，已滤波
 */

#include "stm32h7xx_hal.h"
#include <cmath>
#include <cstdint>
#include <type_traits>  // std::conditional

// ─────────────────────────────────────────────────────────────────────────────
//  EncoderConfig —— 构造时一次性传入，之后不可变
// ─────────────────────────────────────────────────────────────────────────────
struct EncoderConfig {
    float    wheelDiameter_m;   ///< 车轮直径（米）, e.g. 0.065f
    float    gearRatio;         ///< 减速比,          e.g. 20.0f
    uint32_t encoderLines;      ///< 编码器单相线数,  e.g. 13（霍尔）/ 500（GMR）
    float    decodeMultiplier;  ///< 编码器倍频系数，常见 2 或 4
    float    dt_s;              ///< 控制周期（秒）,  e.g. 0.01f
    float    alpha;             ///< IIR 滤波系数 α ∈ (0,1]
                                ///<   小→平滑响应慢  大→跟随响应快  建议起点: 0.3f
};

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder<CounterType>
// ─────────────────────────────────────────────────────────────────────────────
template <typename CounterType = uint16_t>
class Encoder {

    static_assert(
        sizeof(CounterType) == 2 || sizeof(CounterType) == 4,
        "CounterType must be uint16_t (16-bit timer) or uint32_t (32-bit timer)"
    );

    // 差值类型：位宽与 CounterType 一致的有符号整数
    // uint16_t → int16_t,  uint32_t → int32_t
    using DeltaType = typename std::conditional<
        sizeof(CounterType) == 4, int32_t, int16_t
    >::type;

public:
    // ── 构造 ─────────────────────────────────────────────────────────────────
    explicit Encoder(TIM_HandleTypeDef* timer, const EncoderConfig& cfg);

    // ── 硬件初始化（在调度器启动前调用） ────────────────────────────────────
    void begin();

    // ── ★ 核心：每个控制周期调用一次 ────────────────────────────────────────
    /**
     * @brief  统一完成：脉冲采样 → 换算 → IIR 滤波 → 里程累加
     * @return 本周期滤波后的速度（m/s），正转为正，反转为负
     */
    float update();

    // ── 读取接口 ─────────────────────────────────────────────────────────────

    /// 滤波后的瞬时速度（m/s），正转为正，反转为负
    float getFilteredSpeed_mps() const { return filteredSpeed_mps_; }

    /**
     * @brief  绝对累计里程（米），永远 >= 0，倒车也累加（取绝对值）
     * @note   用途：里程计算本周期位移增量时，用相邻两次差值
     *         dLeft = encLeft.getOdometryDistance_m() - prevLeft;
     *         该值体现的是"轮子转过的路程"而非"位移"，符合里程计的物理定义。
     */
    float getOdometryDistance_m() const { return odometryDistance_m_; }

    // ── 重置 ─────────────────────────────────────────────────────────────────
    void reset();

private:
    TIM_HandleTypeDef* htim_;
    const EncoderConfig cfg_;

    CounterType lastCount_;          ///< 上一周期计数器快照，位宽与定时器匹配
    float       pulseToDist_m_;      ///< 预计算：每脉冲对应的线位移（米）
    float       filteredSpeed_mps_;  ///< IIR 滤波器状态量
    float       odometryDistance_m_; ///< 绝对累计里程（恒 >= 0）

    /// 读取当前硬件计数器差值（内部同步 lastCount_）
    DeltaType sampleDelta();
};

// ─────────────────────────────────────────────────────────────────────────────
//  模板实现（必须放在头文件，否则链接器找不到实例化）
// ─────────────────────────────────────────────────────────────────────────────

template <typename CounterType>
Encoder<CounterType>::Encoder(TIM_HandleTypeDef* timer, const EncoderConfig& cfg)
    : htim_(timer)
    , cfg_(cfg)
    , lastCount_(0)
    , filteredSpeed_mps_(0.0f)
    , odometryDistance_m_(0.0f)
{
    // 预计算每脉冲线位移，避免高频任务里重复浮点除法
    // 完整一圈脉冲数 = lines × ratio × decodeMultiplier
    // 每脉冲位移     = π × d / (lines × ratio × decodeMultiplier)
    pulseToDist_m_ = (static_cast<float>(M_PI) * cfg_.wheelDiameter_m)
                     / (static_cast<float>(cfg_.encoderLines)
                        * cfg_.gearRatio
                        * cfg_.decodeMultiplier);
}

template <typename CounterType>
void Encoder<CounterType>::begin() {
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim_, 0);
    lastCount_           = 0;
    filteredSpeed_mps_   = 0.0f;
    odometryDistance_m_  = 0.0f;
}

template <typename CounterType>
typename Encoder<CounterType>::DeltaType Encoder<CounterType>::sampleDelta() {
    // 强转为与定时器位宽匹配的无符号类型，再做无符号减法
    // 无符号减法自然溢出回绕，完美处理计数器翻转：
    //   16-bit 正转: (uint16_t)(0x0005 - 0x0002) =  3  → int16_t: +3  ✓
    //   16-bit 反转: (uint16_t)(0x0002 - 0x0005) = 0xFFFD → int16_t: -3 ✓
    //   32-bit 同理，但差值范围扩展到 ±2^31
    CounterType current = static_cast<CounterType>(__HAL_TIM_GET_COUNTER(htim_));
    DeltaType delta = static_cast<DeltaType>(current - lastCount_);
    lastCount_ = current;
    return delta;
}

template <typename CounterType>
float Encoder<CounterType>::update() {
    DeltaType delta = sampleDelta();

    // 换算瞬时速度：v = delta × (dist/pulse) / dt
    float instantSpeed = static_cast<float>(delta) * pulseToDist_m_ / cfg_.dt_s;

    // IIR 一阶低通滤波：y[n] = α·x[n] + (1-α)·y[n-1]
    filteredSpeed_mps_ = cfg_.alpha * instantSpeed
                       + (1.0f - cfg_.alpha) * filteredSpeed_mps_;

    // ★ Bug 修正：绝对里程永远累加，倒车取绝对值
    // 物理含义："轮子转过了多少路程"，与方向无关
    // 里程计需要的是"有符号位移增量"，由上层 DiffDriveOdometry 用
    // filteredSpeed_mps_ * dt 或相邻差值自行计算，本类只维护绝对里程。
    odometryDistance_m_ += fabsf(static_cast<float>(delta) * pulseToDist_m_);

    return filteredSpeed_mps_;
}

template <typename CounterType>
void Encoder<CounterType>::reset() {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    lastCount_          = 0;
    filteredSpeed_mps_  = 0.0f;
    odometryDistance_m_ = 0.0f;
}