// 底盘控制类，将控制与大脑解耦
#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"
#include "cmsis_os2.h"
#include <FreeRTOS.h>

extern osMessageQueueId_t xSensorQueue;
extern osMessageQueueId_t xMotorQueue;
extern UART_HandleTypeDef huart1;

class ChassisController
{
    public:
        ChassisController();
        void begin();
        void setAction(RobotAction action);
        bool isIdle() const; // 判断是否处于待机状态
        void update(const SensorData& sensor, MotorCommand& cmd); // PID 等

    private:
        RobotAction currentAction = RobotAction::IDLE; // 当前状态
        MoveState moveState = MoveState::STOP; // 运动状态
        TurnDirection nextTurn; // 下一步转向
        SensorData lastSensorData, currentSensorData;
        double yaw; // Z角度，不做标准化处理，随着转向可能叠加到几千（当然是右转更多的前提下），顺时针为正
        double target_yaw; // 目标角度
        MotorCommand ctrl;

        void waitForStartButton();
        void speedHold(MotorCommand* input);
        void gotoStartPlace();
        void forward_withDiff(MotorCommand* ctrl, double baseSpeed,
                            SensorData* sensorData, SensorData* lastSensorData); // 用编码器走直线
        void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set,
                    SensorData* sensorData, SensorData* lastSensorData); // 用右侧距离走直线
        
};