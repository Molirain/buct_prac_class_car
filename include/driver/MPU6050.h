#pragma once
#include "stm32h7xx_hal.h"

class MPU6050 {
private:
    I2C_HandleTypeDef* hi2c;
    // MPU6050 寄存器地址大全
    const uint8_t MPU_ADDR = (0x68 << 1); // 模块接GND的7位地址左移一位
    const uint8_t REG_WHO_AM_I = 0x75;    // 设备ID寄存器
    const uint8_t REG_PWR_MGMT_1 = 0x6B;  // 电源管理寄存器
    const uint8_t REG_SMPLRT_DIV = 0x19;  // 采样率分频寄存器
    const uint8_t REG_CONFIG = 0x1A;      // 配置寄存器
    const uint8_t REG_GYRO_CONFIG = 0x1B; // 陀螺仪配置寄存器
    const uint8_t REG_ACCEL_CONFIG = 0x1C;// 加速度计配置寄存器
    const uint8_t REG_GYRO_ZOUT_H = 0x47; // Z轴角速度高位寄存器
    
    // 核心数据
    float yaw;          // 小车当前的绝对偏航角 (度)
    float gyroZ_offset; // 陀螺仪零点漂移误差
    uint32_t lastTime;  // 上次读取的时间戳
    
    // 封装底层 I2C 操作
    void writeByte(uint8_t reg, uint8_t data);
    void readBytes(uint8_t reg, uint8_t* buffer, uint16_t size);

public:
    // 构造函数
    MPU6050(I2C_HandleTypeDef* i2c_handle);
    
    // 唤醒、配置并校准传感器
    bool begin();
    
    // 核心算法：读取角速度并积分为当前角度
    void update();
    
    // 获取小车当前绝对航向角
    float getYaw();
    
    // 重置航向角为0
    void resetYaw();
};