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

void ChassisController::update(const SensorData& sensor, MotorCommand& cmd, const WallInfo& walls)
{
    currentSensorData = sensor;

    switch(moveState){
        case MoveState::FORWARD:
            forward(&cmd, 30.0, BASE_RIGHT_D, &currentSensorData, &lastSensorData);
            if(currentSensorData.distance[1] < FRONT_STOP_D || shouldTurn(walls)){
                currentAction = RobotAction::IDLE;
                moveState = MoveState::STOP;
            }
            break;
        
        case MoveState::PRE_TURN:
            if(isFirstPreTurn){
                firstPreTurnL = currentSensorData.L[0];
                isFirstPreTurn = false;
            }
            if(currentSensorData.L[0] - firstPreTurnL > 30){ 
                moveState = MoveState::TURN;
                isFirstPreTurn = true;
            } else {
                // 【修正】：把 ctrl 改成 cmd
                forward_withDiff(&cmd, BASE_SPEED, &currentSensorData, &lastSensorData);
            }
            break;

        case MoveState::TURN:
            accum_L[0] += currentSensorData.L[0];
            accum_L[1] += currentSensorData.L[1];
            // 【修正】：把 ctrl 改成 cmd
            if(turn(target_yaw, accum_L, last_error_L, &cmd)){
                moveState = MoveState::AFTER_TURN;
                isFirstAfterTurn = true;
            }
            break;

        case MoveState::AFTER_TURN:
            if(isFirstAfterTurn){
                firstAfterTurnL = currentSensorData.L[0];
                isFirstAfterTurn = false;
            }
            if(currentSensorData.L[0] - firstAfterTurnL > 30){ 
                moveState = MoveState::STOP;
                currentAction = RobotAction::IDLE;
                isFirstAfterTurn = true;
            } else {
                // 【修正】：把 ctrl 改成 cmd
                forward_withDiff(&cmd, BASE_SPEED, &currentSensorData, &lastSensorData);
            }
            break;

        case MoveState::STOP:
            // 【修正】：把 ctrl 改成 cmd
            cmd.speed_percent[0] = 0;
            cmd.speed_percent[1] = 0;
            break;
        
        default:
            break;
    }

    // 【优化】：统一在帧末尾保存历史数据，干掉冗余代码
    lastSensorData = currentSensorData;
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
            cmd.speed_percent[0] = 0;
            cmd.speed_percent[1] = 0;
            osMessageQueuePut(xMotorQueue, &cmd, 0, osWaitForever);
            currentAction = RobotAction::IDLE;
            moveState = MoveState::STOP;
            break;
        }
        forward_withDiff(&cmd, 30.0, &currentSensorData, &lastSensorData);
        osMessageQueuePut(xMotorQueue, &cmd, 0, osWaitForever);
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

bool ChassisController::turn(double target_angle, double* accum_L, double* last_error_L, MotorCommand* ctrl)
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

bool ChassisController::isIdle() const
{
    return currentAction == RobotAction::IDLE;
}

bool ChassisController::shouldTurn(const WallInfo& walls)
{
    if (!walls.leftWall || !walls.rightWall){
        return true;
    }
    return false;
}