// 底盘控制类（脊髓）
#include "module/chassis_controller.h"

void ChassisController::begin()
{
    centerPID.SetTunings(1.5f, 0.05f, 0.5f);
    centerPID.SetOutputLimits(-40.0f, 40.0f);
    centerPID.SetSampleTimeUs(10000);
    centerPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    centerPID.SetMode(QuickPID::Control::automatic);

    osDelay(pdMS_TO_TICKS(3000)); // 等待陀螺仪校准
    waitForStartButton();
    osMessageQueueGet(xSensorQueue, &currentSensorData, NULL, osWaitForever); // 获取初始航向角
    // 初始化 lastSensorData，防止未初始化的垃圾数据干扰第一次PID
    lastSensorData = currentSensorData;
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
            target_yaw = -90.0;
            moveState = MoveState::PRE_TURN;
            break;

        case RobotAction::TURN_RIGHT:
            nextTurn = TurnDirection::RIGHT;
            target_yaw = 90.0;
            moveState = MoveState::PRE_TURN;
            break;

        case RobotAction::TURN_BACK:
            nextTurn = TurnDirection::BACK;
            target_yaw = 180.0;
            moveState = MoveState::TURN;
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
                isFirstTurn = true;
            } else {
                forward_withDiff(&cmd, BASE_SPEED, &currentSensorData, &lastSensorData);
            }
            break;

        case MoveState::TURN:
            if(turn(target_yaw, currentSensorData.Yaw, &cmd)){
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
                forward_withDiff(&cmd, BASE_SPEED, &currentSensorData, &lastSensorData);
            }
            break;

        case MoveState::STOP:
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
        lastSensorData = currentSensorData;
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
    double Kp = 150.0; 
    double Kd = 30.0; 

    // 计算修正量：如果左轮快 (diff > 0)，需要减小左轮，增加右轮
    double correction = Kp * diff + Kd * (diff - last_diff);

    ctrl->speed_percent[0] = baseSpeed - correction;
    ctrl->speed_percent[1] = baseSpeed + correction;

    speedHold(ctrl);
}

void ChassisController::forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData)
{
    static int debounce_cnt = 0;
    static float filtered_R = 100.0f;
    const float LIMIT_DIST = 150.0f;
    const float SPIKE_THRESHOLD = 30.0f;

    float read_R = sensorData->distance[2];

    if (read_R >= LIMIT_DIST) {
        debounce_cnt = 0;
        filtered_R = 200.0f;
    } else {
        if (std::abs(read_R - filtered_R) > SPIKE_THRESHOLD) {
            debounce_cnt++;
            if (debounce_cnt >= 5) {
                filtered_R = read_R;
                debounce_cnt = 0;
            }
        } else {
            filtered_R = read_R;
            debounce_cnt = 0;
        }
    }

    if (filtered_R > 120.0f) {
        forward_withDiff(ctrl, baseSpeed, sensorData, lastSensorData);
        centerPID.Reset();
        return;
    }

    pid_input = filtered_R;
    pid_setpoint = right_distance_set;
    centerPID.Compute();
    
    float correction = pid_output;
    
    auto limit_speed = [](float v) -> float {
        if (v > 50.0f) return 50.0f;
        if (v < -50.0f) return -50.0f;
        return v;
    };

    ctrl->speed_percent[0] = limit_speed(baseSpeed + correction);
    ctrl->speed_percent[1] = limit_speed(baseSpeed - correction);

    speedHold(ctrl);
}

bool ChassisController::turn(double target_angle_diff, float current_yaw, MotorCommand* ctrl)
{
    if (isFirstTurn) {
        start_yaw = current_yaw;
        last_yaw_error = 0.0f;
        isFirstTurn = false;
    }

    float target_abs_yaw = start_yaw + target_angle_diff;
    float error = target_abs_yaw - current_yaw;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (std::abs(error) < 1.5f) {
        ctrl->speed_percent[0] = 0.0f;
        ctrl->speed_percent[1] = 0.0f;
        speedHold(ctrl);
        return true;
    }

    float Kp = 1.2f;
    float Kd = 2.0f;
    float correction = Kp * error + Kd * (error - last_yaw_error);

    auto limit_speed = [](float v) -> float {
        if (v > 35.0f) return 35.0f;
        if (v < -35.0f) return -35.0f;
        return v;
    };

    ctrl->speed_percent[0] = limit_speed(correction);
    ctrl->speed_percent[1] = limit_speed(-correction);

    last_yaw_error = error;
    speedHold(ctrl);
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