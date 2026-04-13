#include "driver/MPU6050.h"

// 构造函数初始化列表
MPU6050::MPU6050(I2C_HandleTypeDef* i2c_handle) 
    : hi2c(i2c_handle), yaw(0.0f), gyroZ_offset(0.0f), lastTime(0) {
    // 创建 FreeRTOS 二值信号量
    dmaSemaphore = osSemaphoreNew(1, 0, NULL);
}

// 封装底层 I2C 写操作
void MPU6050::writeByte(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// 封装底层 I2C 读操作
void MPU6050::readBytes(uint8_t reg, uint8_t* buffer, uint16_t size) {
    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, size, 100);
}

// 初始化：检测ID、配置寄存器、校准零漂
bool MPU6050::begin() {
    uint8_t check;
    readBytes(REG_WHO_AM_I, &check, 1);
    if (check != 0x68) {
        return false; // 检测不到MPU6050
    }
    
    HAL_Delay(50); // 断电上电缓冲
    
    // 核心配置注入
    writeByte(REG_PWR_MGMT_1, 0x00);   // 解除休眠
    writeByte(REG_SMPLRT_DIV, 0x07);   // 采样率分频
    writeByte(REG_CONFIG, 0x06);       // 低通滤波
    writeByte(REG_ACCEL_CONFIG, 0x01); // ±2G 量程
    writeByte(REG_GYRO_CONFIG, 0x18);  // ±2000°/s 量程
    
    HAL_Delay(50);
    
    // 开机静态校准：小车必须静止！
    long totalZ = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t buf[2];
        readBytes(REG_GYRO_ZOUT_H, buf, 2);
        int16_t rawZ = (buf[0] << 8) | buf[1];
        totalZ += rawZ;
        HAL_Delay(2);
    }
    gyroZ_offset = (float)totalZ / 200.0f;
    lastTime = HAL_GetTick();
    return true;
}

// 读取并积分计算航向角
void MPU6050::update() {
    uint8_t buf[2];
    readBytes(REG_GYRO_ZOUT_H, buf, 2);
    
    // 拼接数据
    int16_t rawZ = (buf[0] << 8) | buf[1];
    float realRawZ = (float)rawZ - gyroZ_offset;
    
    // 2000deg/s 量程，灵敏度 16.4 LSB/(deg/s)
    float gyroZ_rate = realRawZ / 16.4f;
    
    // 计算 dt
    uint32_t currentTime = HAL_GetTick();
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;
    
    // 积分
    yaw += gyroZ_rate * dt;
}

// 获取当前角度
float MPU6050::getYaw() {
    return yaw;
}

// 重置角度
void MPU6050::resetYaw() {
    yaw = 0.0f;
}

// 🌟 核心非阻塞读取方法
void MPU6050::update_DMA() {
    // 1. 命令 DMA 去搬砖，CPU 直接撒手不管
    HAL_I2C_Mem_Read_DMA(hi2c, MPU_ADDR, REG_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, dmaRxBuffer, 2);

    // 2. CPU 在这里挂起休眠，死等 DMA 的信号量，绝对不空转消耗算力！
    if (osSemaphoreAcquire(dmaSemaphore, 10) == osOK) { // 最大等10ms超时
        
        // 3. 被唤醒了！说明 DMA 已经把数据完美地放进了 dmaRxBuffer
        int16_t rawZ = (dmaRxBuffer[0] << 8) | dmaRxBuffer[1];
        float realRawZ = (float)rawZ - gyroZ_offset;
        float gyroZ_rate = realRawZ / 16.4f;

        uint32_t currentTime = HAL_GetTick();
        float dt = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        yaw += gyroZ_rate * dt;
    }
}

// 给外部 C 语言回调函数暴露的接口，用于释放信号量
void MPU6050::dmaCompleteCallback() {
    osSemaphoreRelease(dmaSemaphore);
}