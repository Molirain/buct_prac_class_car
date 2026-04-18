/**
 * @file    DiffDriveOdometry.cpp
 * @brief   差速驱动里程计实现
 */

#include "driver/DiffDriveOdometry.h"
#include <cmath>

// ─── 常量 ─────────────────────────────────────────────────────────────────────
static constexpr float kPi    = static_cast<float>(M_PI);
static constexpr float kTwoPi = 2.0f * kPi;

// ─────────────────────────────────────────────────────────────────────────────

DiffDriveOdometry::DiffDriveOdometry(float wheelBase_m)
    : wheelBase_(wheelBase_m)
    , pose_{}           // 零初始化：x=0, y=0, theta=0
{}

void DiffDriveOdometry::update(float dLeft_m, float dRight_m) {
    // 1. 车体中心线位移 & 航向角变化量
    float dCenter = (dLeft_m + dRight_m) * 0.5f;
    float dTheta  = (dRight_m - dLeft_m) / wheelBase_;

    // 2. 用航向角中点插值（龙格-库塔一阶近似）
    //    比直接用当前角精度更高，尤其在大转弯时
    float thetaMid = pose_.theta_rad + dTheta * 0.5f;

    // 3. 更新位姿
    pose_.x_m       += dCenter * cosf(thetaMid);
    pose_.y_m       += dCenter * sinf(thetaMid);
    pose_.theta_rad += dTheta;

    // 4. 角度归一化，防止 theta 无限增长
    normalizeAngle();
}

float DiffDriveOdometry::getTheta_deg() const {
    return pose_.theta_rad * (180.0f / kPi);
}

float DiffDriveOdometry::distanceTo(float targetX_m, float targetY_m) const {
    float dx = targetX_m - pose_.x_m;
    float dy = targetY_m - pose_.y_m;
    return sqrtf(dx * dx + dy * dy);
}

void DiffDriveOdometry::reset() {
    pose_ = Pose2D{};   // 重置为零位姿
}

void DiffDriveOdometry::normalizeAngle() {
    while (pose_.theta_rad >  kPi)  pose_.theta_rad -= kTwoPi;
    while (pose_.theta_rad < -kPi)  pose_.theta_rad += kTwoPi;
}
