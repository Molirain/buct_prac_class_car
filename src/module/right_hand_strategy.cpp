#include "module/right_hand_strategy.h"

RobotAction RightHandStrategy::think(const WallInfo& walls, int x, int y, int heading) {
    if (!walls.rightWall)  return RobotAction::TURN_RIGHT;
    if (!walls.frontWall)  return RobotAction::MOVE_FORWARD;
    if (!walls.leftWall)   return RobotAction::TURN_LEFT;
    return RobotAction::TURN_BACK;
}

void RightHandStrategy::reset() {
    // 无内部状态，无需重置
}