// 底盘控制类（脊髓）
#include "module/chassis_controller.h"

void ChassisController::begin()
{
    speedLPID.SetTunings(400.0f, 2000.0f, 0.10f);  // Kd=0.15 对抗瓷砖缝瞬时差速
    speedLPID.SetOutputLimits(-100.0f, 100.0f);      // ±100 兼容直行+转弯倒车
    speedLPID.SetSampleTimeUs(10000);
    speedLPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    speedLPID.SetMode(QuickPID::Control::automatic);

    speedRPID.SetTunings(400.0f, 2000.0f, 0.10f);  // Kd=0.15 对抗瓷砖缝瞬时差速
    speedRPID.SetOutputLimits(-100.0f, 100.0f);
    speedRPID.SetSampleTimeUs(10000);
    speedRPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    speedRPID.SetMode(QuickPID::Control::automatic);

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
            isFirstForward = true;
            break;

        case RobotAction::TURN_LEFT:
            nextTurn = TurnDirection::LEFT;
            target_yaw = 90.0;   // 左转 → yaw 增（左后+右前）
            moveState = MoveState::PRE_TURN;
            isFirstPreTurn = true;
            break;

        case RobotAction::TURN_RIGHT:
            nextTurn = TurnDirection::RIGHT;
            target_yaw = -90.0;  // 右转 → yaw 减（左前+右后）
            moveState = MoveState::PRE_TURN;
            isFirstPreTurn = true;
            break;

        case RobotAction::TURN_BACK:
            nextTurn = TurnDirection::BACK;
            target_yaw = 180.0;
            moveState = MoveState::TURN;
            isFirstTurn = true;
            break;

        case RobotAction::STOP_FINISH:
            moveState = MoveState::STOP;
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
            forward(&cmd, BASE_FORWORD_SPEED, BASE_RIGHT_D, &currentSensorData, &lastSensorData);
            if(shouldTurn(walls)){
                currentAction = RobotAction::IDLE;
                moveState = MoveState::STOP;
            }
            break;
        
        case MoveState::PRE_TURN:
            if(isFirstPreTurn){
                firstPreTurnL = currentSensorData.L[0];
                firstPreTurnR = currentSensorData.L[1];
                isFirstPreTurn = false;
                speedLPID.Reset();  // 清空上一步遗留的 I 项
                speedRPID.Reset();
            } else {
                // 距离 PID：用剩余距离折算目标速度，再由速度 PID 跟踪
                float distL = currentSensorData.L[0] - firstPreTurnL;
                float distR = currentSensorData.L[1] - firstPreTurnR;
                float distAvg = (distL + distR) * 0.5f;
                float distRemain = PRE_TURN_D - distAvg;

                if (distRemain <= 0.0f && distAvg >= PRE_TURN_D - 5.0f) {
                    // 双条件：剩余距离≤0 且 平均≥目标-5mm（防止编码器跳变误触发）
                    moveState = MoveState::TURN;
                    isFirstPreTurn = true;
                    isFirstTurn = true;
                } else if (distRemain > 0.0f) {
                    // 距离→速度映射：剩余越多越快，自然减速
                    float targetSpeed = distRemain * 0.005f;
                    if (targetSpeed > 0.15f) targetSpeed = 0.15f;
                    if (targetSpeed < 0.03f) targetSpeed = 0.03f;

                    speedL_setpoint = targetSpeed;
                    speedL_input    = currentSensorData.speed[0];
                    speedLPID.Compute();
                    cmd.speed_percent[0] = speedL_output;

                    speedR_setpoint = targetSpeed;
                    speedR_input    = currentSensorData.speed[1];
                    speedRPID.Compute();
                    cmd.speed_percent[1] = speedR_output;
                } else {
                    // distRemain≤0 但 distAvg 未达标：编码器跳变，继续龟速 PID 跟踪
                    speedL_setpoint = 0.05f;
                    speedL_input    = currentSensorData.speed[0];
                    speedLPID.Compute();
                    cmd.speed_percent[0] = speedL_output;

                    speedR_setpoint = 0.05f;
                    speedR_input    = currentSensorData.speed[1];
                    speedRPID.Compute();
                    cmd.speed_percent[1] = speedR_output;
                }
            }
            break;

        case MoveState::TURN:
            if(turn(target_yaw, currentSensorData.Yaw, &cmd)){
                moveState = MoveState::AFTER_TURN;
                isFirstAfterTurn = true;
                
                // 将动作状态置为待机，让大脑再次思考，防止大脑仍然保持上次的 TURN 导致死锁或误判
                currentAction = RobotAction::IDLE; 
            }
            break;

        case MoveState::AFTER_TURN:
            if(isFirstAfterTurn){
                firstAfterTurnL = currentSensorData.L[0];
                firstAfterTurnR = currentSensorData.L[1];
                isFirstAfterTurn = false;
                speedLPID.Reset();  // 清空转弯前遗留的 I 项
                speedRPID.Reset();
            } else {
                // 距离 PID（同 PRE_TURN）
                float distL = currentSensorData.L[0] - firstAfterTurnL;
                float distR = currentSensorData.L[1] - firstAfterTurnR;
                float distAvg = (distL + distR) * 0.5f;
                float distRemain = AFTER_TURN_D - distAvg;

                if (distRemain <= 0.0f && distAvg >= AFTER_TURN_D - 5.0f) {
                    moveState = MoveState::STOP;
                    // AFTER_TURN 结束后，把控制权交还给大脑。由于上面 TURN 结束时已经重置过 currentAction
                    // 这里我们就不再重复设置，只需保持 isIdle() 所需的 IDLE 即可。
                    currentAction = RobotAction::IDLE; 
                    isFirstAfterTurn = true;
                } else if (distRemain > 0.0f) {
                    float targetSpeed = distRemain * 0.005f;
                    if (targetSpeed > 0.15f) targetSpeed = 0.15f;
                    if (targetSpeed < 0.03f) targetSpeed = 0.03f;

                    speedL_setpoint = targetSpeed;
                    speedL_input    = currentSensorData.speed[0];
                    speedLPID.Compute();
                    cmd.speed_percent[0] = speedL_output;

                    speedR_setpoint = targetSpeed;
                    speedR_input    = currentSensorData.speed[1];
                    speedRPID.Compute();
                    cmd.speed_percent[1] = speedR_output;
                } else {
                    // distRemain≤0 但 distAvg 未达标：编码器跳变，继续龟速 PID 跟踪
                    speedL_setpoint = 0.05f;
                    speedL_input    = currentSensorData.speed[0];
                    speedLPID.Compute();
                    cmd.speed_percent[0] = speedL_output;

                    speedR_setpoint = 0.05f;
                    speedR_input    = currentSensorData.speed[1];
                    speedRPID.Compute();
                    cmd.speed_percent[1] = speedR_output;
                }
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
    int i = 0;
    for(;;){
        osMessageQueueGet(xSensorQueue, &currentSensorData, NULL, osWaitForever);
        if(currentSensorData.distance[2] < 30.0){
            i++;
            if(i < 10) continue;
            cmd.speed_percent[0] = 0;
            cmd.speed_percent[1] = 0;
            osMessageQueuePut(xMotorQueue, &cmd, 0, osWaitForever);
            currentAction = RobotAction::IDLE;
            moveState = MoveState::STOP;
            break;
        }
        forward_withDiff(&cmd, 0.05, &currentSensorData, &lastSensorData);
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

void ChassisController::forward_withDiff(MotorCommand* ctrl, double targetSpeed, SensorData* sensorData, SensorData* lastSensorData)
{
    // 速度 PID 控直：左右轮各自独立追踪目标速度 targetSpeed (m/s)
    // PID 输出直接 = PWM 占空比(%)，积分项自然克服电机死区
    float physLeft  = sensorData->speed[0];
    float physRight = sensorData->speed[1];

    speedL_setpoint = (float)targetSpeed;
    speedL_input = physLeft;
    speedLPID.Compute();
    float cmdL = speedL_output;

    speedR_setpoint = (float)targetSpeed;
    speedR_input = physRight;
    speedRPID.Compute();
    float cmdR = speedR_output;

    ctrl->speed_percent[0] = cmdL;
    ctrl->speed_percent[1] = cmdR;
}

void ChassisController::forward(MotorCommand* ctrl, double baseSpeed, double right_distance_set, SensorData* sensorData, SensorData* lastSensorData)
{
    // 编码器直行 + 右墙三区纠正（纯 bang-bang，不用 PID，不碰速度 PID）
    const float WALL_CLOSE = 5.0f;    // <5cm = 太近 → 左转远离
    const float WALL_FAR   = 16.0f;   // >16cm = 太远 → 右转靠近
    const float NUDGE      = 0.04f;   // 速度偏置 (m/s)
    const int   CONFIRM    = 2;       // 连续确认帧数

    static int  closeCount = 0;
    static int  farCount   = 0;
    static float nudge     = 0.0f;

    if (isFirstForward) {
        speedLPID.Reset();
        speedRPID.Reset();
        closeCount = 0;
        farCount = 0;
        nudge = 0.0f;
        isFirstForward = false;
    }

    float read_R = sensorData->distance[2];

    // ---- 三区判断 + 连续确认 ----
    if (read_R >= 3.0f && read_R < 120.0f) {
        if (read_R < WALL_CLOSE) {
            closeCount++;
            farCount = 0;
            if (closeCount >= CONFIRM) nudge = +NUDGE;   // 太近 → 左轮减速右轮加速 = 左转远离
        } else if (read_R > WALL_FAR) {
            farCount++;
            closeCount = 0;
            if (farCount >= CONFIRM) nudge = -NUDGE;      // 太远 → 左轮加速右轮减速 = 右转靠近
        } else {
            // 5~16cm 死区：OK，缓慢衰减上次修正
            closeCount = 0;
            farCount = 0;
            nudge *= 0.5f;
        }
    }
    // 无效读数（<3 或 ≥120）：保持 nudge & 计数器不变

    speedL_setpoint = (float)baseSpeed - nudge;
    speedR_setpoint = (float)baseSpeed + nudge;

    speedL_input = sensorData->speed[0];
    speedLPID.Compute();
    ctrl->speed_percent[0] = speedL_output;

    speedR_input = sensorData->speed[1];
    speedRPID.Compute();
    ctrl->speed_percent[1] = speedR_output;
}

bool ChassisController::turn(double target_angle_diff, float current_yaw, MotorCommand* ctrl)
{
    // ★ 级联 PID：外环角度 PID → 目标速度 → 内环速度 PID（左右轮对称）
    static float turn_error_integral = 0.0f;
    static bool wasNearTarget = false;
    static int  brakeFrame = 0;    // 刹车帧计数（递增）

    if (isFirstTurn) {
        start_yaw = current_yaw;
        turn_error_integral = 0.0f;
        wasNearTarget = false;
        brakeFrame = 0;
        speedLPID.Reset();
        speedRPID.Reset();
    }

    float target_abs_yaw = start_yaw + target_angle_diff;
    float error = target_abs_yaw - current_yaw;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (isFirstTurn) {
        last_yaw_error = error;
        isFirstTurn = false;
    }

    // === 收敛 + 速度阈值刹车：宁可多刹几帧，不可滑过头 ===
    if (std::abs(error) < 2.5f || brakeFrame > 0) {
        if (brakeFrame == 0) {
            speedLPID.Reset();   // 清空转弯累积 I，纯 P+I 制动
            speedRPID.Reset();
        }
        brakeFrame++;

        speedL_setpoint = 0.0f;
        speedR_setpoint = 0.0f;
        speedL_input = currentSensorData.speed[0];
        speedLPID.Compute();
        ctrl->speed_percent[0] = speedL_output;
        speedR_input = currentSensorData.speed[1];
        speedRPID.Compute();
        ctrl->speed_percent[1] = speedR_output;

        // 前 4 帧不管速度（IIR 滞后，报告值偏大），之后等速度降到 ~0.01 以下
        // 最大 25 帧 (250ms) 兜底
        if (brakeFrame >= 4 &&
            std::abs(currentSensorData.speed[0]) < 0.015f &&
            std::abs(currentSensorData.speed[1]) < 0.015f) {
            brakeFrame = 0;
            ctrl->speed_percent[0] = 0.0f;
            ctrl->speed_percent[1] = 0.0f;
            return true;
        }
        if (brakeFrame >= 25) {
            brakeFrame = 0;
            ctrl->speed_percent[0] = 0.0f;
            ctrl->speed_percent[1] = 0.0f;
            return true;
        }
        return false;
    }

    // === 外环：角度 PID（保持原有调优）===
    float Kp = 0.45f;
    float Ki = 0.25f;
    float Kd = 0.10f;

    float delta_error = error - last_yaw_error;
    while (delta_error >  180.0f) delta_error -= 360.0f;
    while (delta_error < -180.0f) delta_error += 360.0f;

    if ((error > 0 && last_yaw_error < 0) || (error < 0 && last_yaw_error > 0)) {
        turn_error_integral = 0.0f;
    }

    turn_error_integral += error;
    if (turn_error_integral >  100.0f) turn_error_integral =  100.0f;
    if (turn_error_integral < -100.0f) turn_error_integral = -100.0f;

    bool nearTarget = (std::abs(error) < 15.0f);
    if (nearTarget && !wasNearTarget) {
        turn_error_integral = 0.0f;
    }
    wasNearTarget = nearTarget;

    float correction = Kp * error + Ki * turn_error_integral + Kd * delta_error;

    // === 角度 PID 输出 → 目标速度 (m/s) ===
    // correction 典型范围 0~65，映射到 0~0.20 m/s（降低上限减惯性过冲）
    float targetSpeed = std::abs(correction) * 0.004f;
    if (targetSpeed > 0.20f) targetSpeed = 0.20f;
    if (nearTarget && targetSpeed > 0.08f) targetSpeed = 0.08f;  // 减速区软限
    if (targetSpeed < 0.05f) targetSpeed = 0.05f;  // 保底 ≥ 0.05，编码器 ~1.7 脉冲/帧

    // correction>0 → 左轮倒车、右轮前进 → yaw 增大（左转）
    speedL_setpoint = (correction > 0) ? -targetSpeed : targetSpeed;
    speedR_setpoint = (correction > 0) ? targetSpeed : -targetSpeed;

    // === 内环：速度 PID，左右轮对称跟踪 ===
    speedL_input = currentSensorData.speed[0];
    speedLPID.Compute();
    ctrl->speed_percent[0] = speedL_output;

    speedR_input = currentSensorData.speed[1];
    speedRPID.Compute();
    ctrl->speed_percent[1] = speedR_output;

    last_yaw_error = error;
    return false;
}


bool ChassisController::isIdle() const
{
    return currentAction == RobotAction::IDLE;
}

bool ChassisController::shouldTurn(const WallInfo& walls)
{
    if (walls.frontWall || !walls.rightWall){
        return true;
    }
    return false;
}