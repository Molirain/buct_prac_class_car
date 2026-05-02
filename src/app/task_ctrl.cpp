#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "module/chassis_controller.h"
#include "module/sensor_filter.h"
#include "module/right_hand_strategy.h"
#include "module/maze_strategy.h" 
#include "config.h"

extern osMessageQueueId_t xMotorQueue;
extern osMessageQueueId_t xSensorQueue;
extern UART_HandleTypeDef huart1;

// 模块实例化
static SensorFilter filter;
static ChassisController chassis;
static RightHandStrategy rightHandBrain; 

void AppTaskCtrl(void *argument) {
    SensorData sensor;
    WallInfo walls;
    MotorCommand cmd;

    // 逻辑坐标与朝向追踪（目前的大脑 think 函数不需要，可以传 0）
    int currentX = 0;
    int currentY = 0;
    int currentHeading = 0; // 0:北, 1:东, 2:南, 3:西

    // 多态指针
    // 测试算法，改成 = &smartBrain; 即可切换
    MazeStrategy* brain = &rightHandBrain; 

    // 陀螺仪校准延迟、等待按键按下、以及进入初始格子
    chassis.begin(); 

    // 周期由 sensor_task 保障
    for(;;){
        if (osMessageQueueGet(xSensorQueue, &sensor, NULL, osWaitForever) == osOK) {
            walls = filter.update(sensor);
            
            // 大脑思考
            if (chassis.isIdle()) {
                // 思考下一步
                RobotAction action = brain->think(walls, currentX, currentY, currentHeading);
                chassis.setAction(action); // 下达指令

                // 预留 FloodFill，根据 action 更新 currentX, currentY 和 currentHeading
            }

            // PID and so on
            chassis.update(sensor, cmd, walls);
            osMessageQueuePut(xMotorQueue, &cmd, 0, osWaitForever);
        }
    }
}