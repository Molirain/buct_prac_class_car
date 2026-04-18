#pragma once
/**
 * @file    DiffDriveOdometry.h
 * @brief   差速驱动底盘里程计（X / Y / θ 航迹推算）
 *
 * 职责边界（单一职责原则）：
 *   ✅ 本类负责：接收左右轮有符号位移增量，推算小车在二维平面的位姿
 *               (x, y, θ) ，对外提供只读访问接口。
 *   ❌ 本类不负责：读硬件寄存器、IIR 滤波——那是 Encoder 的职责。
 *
 * 差速驱动运动学模型：
 *
 *   输入（每周期）：
 *     dL = 左轮有符号线位移增量（米），前进为正，倒车为负
 *     dR = 右轮有符号线位移增量（米），前进为正，倒车为负
 *     B  = 两轮接地点间距（轴距，米）
 *
 *   推导：
 *     dCenter  = (dL + dR) / 2        // 车体中心线位移
 *     dTheta   = (dR - dL) / B        // 航向角变化量（弧度）
 *     θ_mid    = θ + dTheta / 2       // 中点角（比直接用当前角精度更高）
 *     x       += dCenter × cos(θ_mid)
 *     y       += dCenter × sin(θ_mid)
 *     θ       += dTheta
 *
 * 使用示例：
 *   #include "DiffDriveOdometry.h"
 *   DiffDriveOdometry odom(0.145f);   // 轴距 145mm
 *   // 每 10ms：
 *   float dL = encLeft.getFilteredSpeed_mps()  * 0.01f;
 *   float dR = encRight.getFilteredSpeed_mps() * 0.01f;
 *   odom.update(dL, dR);
 *   float x = odom.getX_m();
 *   float y = odom.getY_m();
 */

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
//  Pose2D —— 二维位姿（x, y, θ）
//  独立定义在此头文件，不依赖 Encoder.h，保持低耦合
// ─────────────────────────────────────────────────────────────────────────────
struct Pose2D {
    float x_m       = 0.0f;   ///< 相对起点 X 坐标（米）
    float y_m       = 0.0f;   ///< 相对起点 Y 坐标（米）
    float theta_rad = 0.0f;   ///< 朝向角（弧度），以初始方向为 0，逆时针为正
};

// ─────────────────────────────────────────────────────────────────────────────
//  DiffDriveOdometry
// ─────────────────────────────────────────────────────────────────────────────
class DiffDriveOdometry {
public:
    /**
     * @param wheelBase_m  左右车轮接地点间距（米）
     *                     测量方法：两轮胎接地中心线之间的距离
     */
    explicit DiffDriveOdometry(float wheelBase_m);

    /**
     * @brief  每个控制周期调用一次
     * @param dLeft_m   左轮本周期有符号线位移增量（米），前进正，倒车负
     * @param dRight_m  右轮本周期有符号线位移增量（米），前进正，倒车负
     *
     * @note  如何获取 dLeft_m / dRight_m（两种等价方法）：
     *
     *   方法 A（推荐，误差更小）：
     *     用滤波后速度 × dt，速度已经过 IIR 平滑，噪声少
     *     float dL = encLeft.getFilteredSpeed_mps()  * dt_s;
     *     float dR = encRight.getFilteredSpeed_mps() * dt_s;
     *
     *   方法 B：
     *     用 getRawDelta() 转换后的瞬时位移，未滤波但无相位延迟
     *     （适合需要极低延迟的场合）
     */
    void update(float dLeft_m, float dRight_m);

    // ── 读取当前位姿 ─────────────────────────────────────────────────────────
    const Pose2D& getPose()       const { return pose_; }
    float         getX_m()        const { return pose_.x_m; }
    float         getY_m()        const { return pose_.y_m; }
    float         getTheta_rad()  const { return pose_.theta_rad; }
    float         getTheta_deg()  const;

    /**
     * @brief  到目标点的欧氏距离（米），迷宫导航中判断是否到格有用
     */
    float distanceTo(float targetX_m, float targetY_m) const;

    // ── 重置位姿到原点 ───────────────────────────────────────────────────────
    void reset();

private:
    float  wheelBase_;   ///< 轴距（米），构造后不可变
    Pose2D pose_;        ///< 当前位姿

    /// 将 theta 归一化到 [-π, π]，防止数值长期漂移
    void normalizeAngle();
};
