// 底盘控制类，将控制与大脑解耦
#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"
#include "cmsis_os2.h"
#include <FreeRTOS.h>
#include "config.h"
#include <QuickPID.h>

extern osMessageQueueId_t xSensorQueue;
extern osMessageQueueId_t xMotorQueue;
extern UART_HandleTypeDef huart1;

class ChassisController
{
    public:
        void begin();
        void setAction(RobotAction action);
        bool isIdle() const; // 判断是否处于待机状态
        void update(const SensorData& sensor, MotorCommand& cmd, const WallInfo& walls); // PID 等

    private:
        float pid_input = 0.0f;
        float pid_output = 0.0f;
        float pid_setpoint = 0.0f;
        QuickPID centerPID = QuickPID(&pid_input, &pid_output, &pid_setpoint);

        RobotAction currentAction = RobotAction::IDLE; // 当前状态
        MoveState moveState = MoveState::STOP; // 运动状态
        TurnDirection nextTurn; // 下一步转向
        SensorData lastSensorData, currentSensorData;
        double yaw; // Z角度，不做标准化处理，随着转向可能叠加到几千（当然是右转更多的前提下），顺时针为正
        double target_yaw; // 目标角度
        MotorCommand cmd;
        bool isFirstAfterTurn = true;
        bool isFirstPreTurn = true;
        bool isFirstTurn = true;
        float start_yaw = 0.0f;
        float last_yaw_error = 0.0f;
        double firstAfterTurnL = 0; // 转弯后前进的起始里程
        double firstPreTurnL = 0; // 转弯前前进的起始里程

        void waitForStartButton();
        void speedHold(MotorCommand* input);
        void gotoStartPlace();
        void forward_withDiff(MotorCommand* ctrl, double baseSpeed,
                            SensorData* sensorData, SensorData* lastSensorData); // 用编码器走直线
        void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set,
                    SensorData* sensorData, SensorData* lastSensorData); // 用右侧距离走直线
        bool turn(double target_angle_diff, float current_yaw, MotorCommand* ctrl); // 转弯
        bool shouldTurn(const WallInfo& walls); // 判断是否需要转弯
};