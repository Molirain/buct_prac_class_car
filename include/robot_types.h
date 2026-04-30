#pragma once

// 1. 传感器获取到的墙壁信息（大脑只看这个，不关心具体的毫米数）
struct WallInfo {
    bool leftWall;
    bool frontWall;
    bool rightWall;
};

// 2. 大脑下达给双腿的标准指令
enum class RobotAction {
    IDLE,           // 发呆/待机
    MOVE_FORWARD,   // 往前走（走到下一个路口）
    TURN_LEFT,      // 原地左转 90 度
    TURN_RIGHT,     // 原地右转 90 度
    TURN_BACK,      // 原地掉头 180 度
    STOP_FINISH     // 到达终点
};

// 有限状态机参量
enum class MoveState{
    FORWARD, // 直行
    PRE_TURN, // 转弯前需要前进一点点
    TURN, // 转弯中
    AFTER_TURN, // 转弯后也需要前进一点点
    STOP // 停止，用于开始前和结束后
};

// 下一步的转弯方向
enum class TurnDirection{
    LEFT,
    RIGHT,
    BACK
};