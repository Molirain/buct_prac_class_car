#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "module/chassis_controller.h"
#include "module/sensor_filter.h"
#include "module/right_hand_strategy.h"
#include "module/left_hand_strategy.h"
#include "module/maze_strategy.h" 
#include "config.h"

extern osMessageQueueId_t xMotorQueue;
extern osMessageQueueId_t xSensorQueue;
extern UART_HandleTypeDef huart1;

// 模块实例化
static SensorFilter filter;
static ChassisController chassis;
static RightHandStrategy rightHandBrain;
static LeftHandStrategy  leftHandBrain; 

WallInfo g_walls; // 供串口任务直接读取的全局变量
RobotAction g_lastAction = RobotAction::IDLE; // 供串口打印最后一次大脑决断

void AppTaskCtrl(void *argument) {
    SensorData sensor;
    WallInfo walls;
    MotorCommand cmd;

    // 逻辑坐标与朝向追踪（目前的大脑 think 函数不需要，可以传 0）
    int currentX = 0;
    int currentY = 0;
    int currentHeading = 0; // 0:北, 1:东, 2:南, 3:西

    // 多态指针 — 根据按下的是 START（右手）还是 SEND（左手）来选择策略
    MazeStrategy* brain = nullptr;

    // 陀螺仪校准延迟、等待按键按下、以及进入初始格子；返回按了哪个键
    StartButton which = chassis.begin();
    if (which == StartButton::SEND) {
        brain = &leftHandBrain;
        chassis.setLeftHandMode(true);   // 告诉底盘：左手法则，检测左墙缺口
        HAL_UART_Transmit(&huart1, (uint8_t*)"Strategy: Left-Hand Rule\r\n", 26, 100);
    } else {
        brain = &rightHandBrain;
        chassis.setLeftHandMode(false);  // 告诉底盘：右手法则，检测右墙缺口
        HAL_UART_Transmit(&huart1, (uint8_t*)"Strategy: Right-Hand Rule\r\n", 27, 100);
    }

    // gotoStartPlace 后车身可能没摆正。持续读传感器直到右墙被连续确认 5 帧，最多等 60 帧（6秒）。
    int primeStable = 0;
    for (int primeCount = 0; primeCount < 60; primeCount++) {
        osMessageQueueGet(xSensorQueue, &sensor, NULL, osWaitForever);
        walls = filter.update(sensor);
        g_walls = walls;
        if (walls.rightWall) {
            primeStable++;
            if (primeStable >= 5) break;
        } else {
            primeStable = 0;
        }
    }
    // 起步防护：forward 开始后前 30 帧不允许因右墙丢失而右转（防止车身没摆正导致的误判）
    int startupGuard = 0;

    // 周期由 sensor_task 保障
    for(;;){
        if (osMessageQueueGet(xSensorQueue, &sensor, NULL, osWaitForever) == osOK) {
            walls = filter.update(sensor);
            g_walls = walls; // 更新全局状态供 comm_task 打印
            
            // 大脑思考
            if (chassis.isIdle()) {
                // 思考下一步
                RobotAction action = brain->think(walls, currentX, currentY, currentHeading);

                // 【防误判补丁】：TURN_BACK 时检查前测距真实值
                if (action == RobotAction::TURN_BACK) {
                    if (sensor.distance[0] > 100.0f) {
                        action = RobotAction::MOVE_FORWARD;
                        walls.frontWall = false;
                        g_walls.frontWall = false;
                    }
                }

                // 【起步防护】：forward 刚开始 30 帧内，转向极可能是车身未摆正导致的。
                if (startupGuard < 30) {
                    if (action == RobotAction::TURN_RIGHT && !walls.leftWall) {
                        action = RobotAction::MOVE_FORWARD;   // 右手法则：右转误判
                    }
                    if (action == RobotAction::TURN_LEFT && !walls.rightWall) {
                        action = RobotAction::MOVE_FORWARD;   // 左手法则：左转误判
                    }
                }

                if (action == RobotAction::MOVE_FORWARD) {
                    startupGuard = 0; // 新直行段，重置防护计数器
                }

                g_lastAction = action; // 记录给串口打印
                chassis.setAction(action); // 下达指令
            }
            
            // 直行期间递增防护计数器
            if (!chassis.isIdle()) {
                startupGuard++;
            }

            // PID and so on
            chassis.update(sensor, cmd, walls);
            osMessageQueuePut(xMotorQueue, &cmd, 0, osWaitForever);
        }
    }
}