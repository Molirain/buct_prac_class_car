// 底盘控制类，将控制与大脑解耦
#pragma once
#include "robot_types.h"
#include "app/tasks.h"
#include "main.h"
#include "cmsis_os2.h"
#include <FreeRTOS.h>
#include "config.h"
#include <QuickPID.h>
#include <cmath>

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
        float speedL_input = 0.0f;
        float speedL_output = 0.0f;
        float speedL_setpoint = 0.0f;
        QuickPID speedLPID = QuickPID(&speedL_input, &speedL_output, &speedL_setpoint);

        float speedR_input = 0.0f;
        float speedR_output = 0.0f;
        float speedR_setpoint = 0.0f;
        QuickPID speedRPID = QuickPID(&speedR_input, &speedR_output, &speedR_setpoint);

        float straightBaseSpeed = 0.05f; // 直行目标速度 m/s，可通过 CLI 动态调节
        bool straightMode = false; // 编码器直行模式标志

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
        bool isFirstForward = true;
        float start_yaw = 0.0f;
        float last_yaw_error = 0.0f;
        double firstAfterTurnL = 0; // 转弯后前进的起始里程（左）
        double firstAfterTurnR = 0; // 转弯后前进的起始里程（右）
        double firstPreTurnL = 0; // 转弯前前进的起始里程（左）
        double firstPreTurnR = 0; // 转弯前前进的起始里程（右）

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