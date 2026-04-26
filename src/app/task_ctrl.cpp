#include "app/tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t xMotorQueue;
extern osMessageQueueId_t xSensorQueue;
extern UART_HandleTypeDef huart1;

// 有限状态机参量
enum MOVE_STATE{
    FORWARD, // 直行
    PRE_TURN, // 转弯前需要前进一点点
    TURN, // 转弯中
    AFTER_TURN, // 转弯后也需要前进一点点
    STOP // 停止，用于开始前和结束后
};

// 下一步的转弯方向
enum TURN_DIRECTION{
    LEFT,
    RIGHT,
    BACK
};

MOVE_STATE moveState = STOP;
TURN_DIRECTION nextTurn;

void AppTaskCtrl(void *argument) {
    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准

    bool isOver = false; // 是否完成迷宫，用于后续完善 send 功能
    SensorData sensorData;
    SensorData lastSensorData;
    double yaw; // Z角度，不做标准化处理，随着转向可能叠加到几千（当然是右转更多的前提下），顺时针为正
    osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever); // 获取初始航向角
    yaw = sensorData.Yaw;
    double target_yaw; // 目标角度
    double accum_L[2] = {0, 0};
    double last_error_L[2] = {0, 0};
    MotorCommand ctrl;
    double baseSpeed = BASE_SPEED; // 基础速度，后续完善 PID 时会用到
    double right_dist_set = BASE_RIGHT_D; // 靠右的距离
    uint8_t right_over_num = 0; // 右侧连续三次超出距离才右转，避免误判
    uint8_t left_over_num = 0; // 左侧连续三次超出距离才左转，避免误判
    uint8_t back_over_num = 0; // 前方连续三次超出距离才后转，避免误判
    uint8_t isOver_num = 0; // 超过三次到达停止条件才暂停

    bool isFirstAfterTurn = true;
    uint64_t firstAfterTurnL = 0;

    bool isFirstPreTurn = true;
    uint64_t firstPreTurnL = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)"TASK Ctrl Start!\r\n", 18, 100);

    // 等待开始按钮按下
    for(;;){
        if(HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
            osDelay(20);
            if (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
                while(HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
                    osDelay(10);
                }
                HAL_UART_Transmit(&huart1, (uint8_t*)"Button Pressed! Go!\r\n", 21, 100);
                break;
            }
        }
        osDelay(10);
    }

    // 清空不需要的历史数据，代替 reset 以防止 xQueueReset 导致的死锁
    SensorData dumpSensor;
    while(osMessageQueueGet(xSensorQueue, &dumpSensor, NULL, 0) == osOK);
    MotorCommand dumpMotor;
    while(osMessageQueueGet(xMotorQueue, &dumpMotor, NULL, 0) == osOK);
    
    // 初始化 lastSensorData，防止未初始化的垃圾数据干扰第一次PID
    osMessageQueueGet(xSensorQueue, &lastSensorData, NULL, osWaitForever);
    
    moveState = FORWARD;

    for(;;){
        if(isOver){
            ctrl.speed_percent[0] = 0;
            ctrl.speed_percent[1] = 1;
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            osDelay(1000);
        }
        // 有限状态机
        switch (moveState)
        {
        case FORWARD:
            if(right_over_num >= 3){
                moveState = PRE_TURN;
                nextTurn = RIGHT;
                isFirstPreTurn = true;
                target_yaw = 90.0;
                break;
            }
            if(left_over_num >= 3){
                moveState = PRE_TURN;
                nextTurn = LEFT;
                isFirstPreTurn = true;
                target_yaw = -90.0;
                break;
            }
            if(back_over_num >= 3){
                moveState = PRE_TURN;
                nextTurn = BACK;
                isFirstPreTurn = true;
                target_yaw = 180.0;
                break;
            }
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            yaw = sensorData.Yaw;
            if(sensorData.distance[2] > 16){
                right_over_num++;
            } else {
                right_over_num = 0;
            }
            if(sensorData.distance[1] < 18){
                if(sensorData.distance[0] > 30){
                    left_over_num++;
                } else {
                    left_over_num = 0;
                    back_over_num++;
                }
            } else {
                left_over_num = 0;
                back_over_num = 0;
            }
            forward(&ctrl, baseSpeed, right_dist_set, &sensorData, &lastSensorData);
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            lastSensorData = sensorData;

            break;
        
        case PRE_TURN:
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            if(isFirstPreTurn){
                firstPreTurnL = sensorData.L[0];
                isFirstPreTurn = false;
            }
            if(sensorData.L[0] - firstPreTurnL > 30){ // 前进至少30mm再转弯，避免转弯不彻底导致误判
                moveState = TURN;
            }
            forward_withDiff(&ctrl, baseSpeed, &sensorData, &lastSensorData);
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            lastSensorData = sensorData;
            break;

        case TURN:
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            accum_L[0] += sensorData.L[0];
            accum_L[1] += sensorData.L[1];

            if(turn(target_yaw, accum_L, last_error_L, &ctrl)){
                moveState = AFTER_TURN;
                isFirstAfterTurn = true;
                break;
            }
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            break;

        case AFTER_TURN:
            osMessageQueueGet(xSensorQueue, &sensorData, NULL, osWaitForever);
            if(isFirstAfterTurn){
                firstAfterTurnL = sensorData.L[0];
                isFirstAfterTurn = false;
            }
            if(sensorData.L[0] - firstAfterTurnL > 30){ // 转弯后前进至少30mm再进入正常巡线，避免转弯不彻底导致误判
                moveState = FORWARD;
            }
            forward_withDiff(&ctrl, baseSpeed, &sensorData, &lastSensorData);
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            lastSensorData = sensorData;
            break;

        case STOP:
            ctrl.speed_percent[0] = 0;
            ctrl.speed_percent[1] = 0;
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            break;
        
        default:
            // 串口传报错
            break;
        }
        // 延时让出CPU
        osDelay(20);
    }
}

void speedHold(MotorCommand* input)
{
    double left = input->speed_percent[0];
    double right = input->speed_percent[1];
    if(left < 0) left = -left;
    if(right < 0) right = -right;
    double max = left > right ? left : right;
    if(max > 100){
        input->speed_percent[0] = input->speed_percent[0] / max * 100;
        input->speed_percent[1] = input->speed_percent[1] / max * 100;
    }
}

void forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData)
{
    double error = sensorData->distance[2] - right_distance_set;
    double last_error = lastSensorData->distance[2] - right_distance_set;
    double Kp = 2.0;
    double Kd = 1.0;

    ctrl->speed_percent[0] = baseSpeed + Kp * error + Kd * (error - last_error);
    ctrl->speed_percent[1] = baseSpeed - Kp * error - Kd * (error - last_error);
    speedHold(ctrl);
}

void forward_withDiff(MotorCommand* ctrl, double baseSpeed, SensorData* sensorData, SensorData* lastSensorData)
{
    // 目标是让左右轮速度一致，保持直线行驶
    // 取差值为：左轮速度 - 右轮速度
    double diff = sensorData->speed[0] - sensorData->speed[1];
    double last_diff = lastSensorData->speed[0] - lastSensorData->speed[1];

    // PD 控制参数（针对速度差值，speed 单位如果为 m/s 值较小，Kp可能需要较大，具体请根据实际调参）
    double Kp = 50.0; 
    double Kd = 10.0; 

    // 计算修正量：如果左轮快 (diff > 0)，需要减小左轮，增加右轮
    double correction = Kp * diff + Kd * (diff - last_diff);

    ctrl->speed_percent[0] = baseSpeed - correction;
    ctrl->speed_percent[1] = baseSpeed + correction;

    speedHold(ctrl);
}

bool turn(double target_angle, double* accum_L, double* last_error_L, MotorCommand* ctrl)
{
    // 轮距为145mm，将目标角度（顺时针为正）转换为左右轮距离
    double track_width = 145.0;
    double target_L = (3.1415926 * track_width * target_angle) / 360.0;

    // 左轮顺时针转需要向前走（正），右轮需要向后走（负）
    double error_L0 = target_L - accum_L[0];  // 左轮误差
    double error_L1 = -target_L - accum_L[1]; // 右轮误差

    // 如果左右轮误差都在2mm以内，认为转弯完成
    if (error_L0 < 2.0 && error_L0 > -2.0 && error_L1 < 2.0 && error_L1 > -2.0) {
        return true;
    }

    // PD 控制
    double Kp = 1.0; // P 参数，根据实际需要调节
    double Kd = 0.5; // D 参数

    ctrl->speed_percent[0] = Kp * error_L0 + Kd * (error_L0 - last_error_L[0]);
    ctrl->speed_percent[1] = Kp * error_L1 + Kd * (error_L1 - last_error_L[1]);

    speedHold(ctrl);

    last_error_L[0] = error_L0;
    last_error_L[1] = error_L1;

    return false;
}