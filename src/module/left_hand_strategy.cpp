#include "module/left_hand_strategy.h"

RobotAction LeftHandStrategy::think(const WallInfo& walls, int x, int y, int heading) {
    if (!walls.leftWall)   return RobotAction::TURN_LEFT;
    if (!walls.frontWall)  return RobotAction::MOVE_FORWARD;
    if (!walls.rightWall)  return RobotAction::TURN_RIGHT;
    return RobotAction::TURN_BACK;
}

void LeftHandStrategy::reset() {
    // 无内部状态，无需重置
}
