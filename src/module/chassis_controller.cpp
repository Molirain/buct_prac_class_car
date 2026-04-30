// 底盘控制类（脊髓）
#include "module/chassis_controller.h"

void ChassisController::begin()
{
    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准
    waitForStartButton();
    osMessageQueueGet(xSensorQueue, &currentSensorData, NULL, osWaitForever); // 获取初始航向角
    // 初始化 lastSensorData，防止未初始化的垃圾数据干扰第一次PID
    osMessageQueueGet(xSensorQueue, &lastSensorData, NULL, osWaitForever);
    gotoStartPlace();
}

void ChassisController::setAction(RobotAction action)
{
    currentAction = action;

    switch (action){
        case RobotAction::MOVE_FORWARD:
            moveState = MoveState::FORWARD;
            break;

        case RobotAction::TURN_LEFT:
            nextTurn = TurnDirection::LEFT;
            moveState = MoveState::PRE_TURN;
            break;

        case RobotAction::TURN_RIGHT:
            nextTurn = TurnDirection::RIGHT;
            moveState = MoveState::PRE_TURN;
            break;

        case RobotAction::TURN_BACK:
            nextTurn = TurnDirection::BACK;
            moveState = MoveState::PRE_TURN;
            break;

        default:
            moveState = MoveState::STOP;
            break;
    }
}

void ChassisController::update(const SensorData& sensor, MotorCommand& cmd)
{
    currentSensorData = sensor;

    switch(moveState){
        // to be writen
    }
}

void ChassisController::waitForStartButton()
{    
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
}

void ChassisController::gotoStartPlace()
{
    for(;;){
        osMessageQueueGet(xSensorQueue, &currentSensorData, NULL, osWaitForever);
        if(currentSensorData.distance[2] < 30.0){
            ctrl.speed_percent[0] = 0;
            ctrl.speed_percent[1] = 0;
            osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
            currentAction = RobotAction::IDLE;
            moveState = MoveState::STOP;
            break;
        }
        forward_withDiff(&ctrl, 30.0, &currentSensorData, &lastSensorData);
        osMessageQueuePut(xMotorQueue, &ctrl, 0, osWaitForever);
    }
}

void ChassisController::speedHold(MotorCommand* input)
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

void ChassisController::forward_withDiff(MotorCommand* ctrl, double baseSpeed, SensorData* sensorData, SensorData* lastSensorData)
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

void ChassisController::forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData)
{
    double error = sensorData->distance[2] - right_distance_set;
    double last_error = lastSensorData->distance[2] - right_distance_set;
    double Kp = 2.0;
    double Kd = 1.0;

    ctrl->speed_percent[0] = baseSpeed + Kp * error + Kd * (error - last_error);
    ctrl->speed_percent[1] = baseSpeed - Kp * error - Kd * (error - last_error);
    speedHold(ctrl);
}

bool ChassisController::isIdle() const
{
    return currentAction == RobotAction::IDLE;
}