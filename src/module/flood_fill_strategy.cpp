#include "module/flood_fill_strategy.h"
#include <cstring>

// ─────────────────────────────────────────────
//  构造 & reset
// ─────────────────────────────────────────────

FloodFillStrategy::FloodFillStrategy() {
    reset();
}

void FloodFillStrategy::reset() {
    // 清空墙壁（仅保留边界）
    memset(hWalls, false, sizeof(hWalls));
    memset(vWalls, false, sizeof(vWalls));
    initBoundaryWalls();

    // 初始化距离图为INF
    memset(distances, INF, sizeof(distances));

    queue.clear();

    // 从终点做一次初始灌水
    floodFill();
}

void FloodFillStrategy::initBoundaryWalls() {
    for (int i = 0; i < SIZE; ++i) {
        hWalls[0][i]    = true;   // 南边界
        hWalls[SIZE][i] = true;   // 北边界
        vWalls[i][0]    = true;   // 西边界
        vWalls[i][SIZE] = true;   // 东边界
    }
}

// ─────────────────────────────────────────────
//  墙壁更新
// ─────────────────────────────────────────────

// relativeDir: 0=前 1=右 2=后 3=左（相对机器人朝向）
// heading:     0北 1东 2南 3西
int FloodFillStrategy::toAbsolute(int heading, int relativeDir) {
    // 前方=heading，右方=(heading+1)%4，后方=(heading+2)%4，左方=(heading+3)%4
    return (heading + relativeDir) % 4;
}

void FloodFillStrategy::updateWalls(int x, int y, int heading, const WallInfo& walls) {
    // 传感器：left=相对左=绝对(heading+3)%4，front=heading，right=(heading+1)%4
    auto setWall = [&](int absDir, bool present) {
        if (!present) return;
        switch (absDir) {
            case 0: hWalls[y + 1][x] = true; break;  // 北
            case 1: vWalls[y][x + 1] = true; break;  // 东
            case 2: hWalls[y][x]     = true; break;  // 南
            case 3: vWalls[y][x]     = true; break;  // 西
        }
    };

    setWall(toAbsolute(heading, 3), walls.leftWall);
    setWall(toAbsolute(heading, 0), walls.frontWall);
    setWall(toAbsolute(heading, 1), walls.rightWall);
}

// ─────────────────────────────────────────────
//  Flood Fill（BFS 从终点向外灌水）
// ─────────────────────────────────────────────

void FloodFillStrategy::floodFill() {
    // 重置全图为INF
    memset(distances, INF, sizeof(distances));
    queue.clear();

    // 终点区域距离=0，入队
    const int goals[4][2] = {{7,7},{7,8},{8,7},{8,8}};
    for (auto& g : goals) {
        int gx = g[0], gy = g[1];
        distances[gy][gx] = 0;
        queue.push(gx, gy);
    }

    // BFS 蔓延
    while (!queue.empty()) {
        int cx, cy;
        queue.pop(cx, cy);
        uint8_t nextDist = distances[cy][cx] + 1;

        // 四个邻居：北东南西
        const int dx[4] = {0, 1,  0, -1};
        const int dy[4] = {1, 0, -1,  0};

        for (int dir = 0; dir < 4; ++dir) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];

            // 越界检查
            if (nx < 0 || nx >= SIZE || ny < 0 || ny >= SIZE) continue;

            // 墙壁检查（从当前格子看向邻居方向）
            bool blocked = false;
            switch (dir) {
                case 0: blocked = hasNorthWall(cx, cy); break;
                case 1: blocked = hasEastWall (cx, cy); break;
                case 2: blocked = hasSouthWall(cx, cy); break;
                case 3: blocked = hasWestWall (cx, cy); break;
            }
            if (blocked) continue;

            // 仅在能改善距离时更新（避免重复入队）
            if (distances[ny][nx] > nextDist) {
                distances[ny][nx] = nextDist;
                queue.push(nx, ny);
            }
        }
    }
}

// ─────────────────────────────────────────────
//  动作决策
// ─────────────────────────────────────────────

RobotAction FloodFillStrategy::dirToAction(int targetAbsDir, int heading) {
    int diff = (targetAbsDir - heading + 4) % 4;
    switch (diff) {
        case 0: return RobotAction::MOVE_FORWARD;
        case 1: return RobotAction::TURN_RIGHT;
        case 2: return RobotAction::TURN_BACK;
        case 3: return RobotAction::TURN_LEFT;
        default: return RobotAction::IDLE;
    }
}

RobotAction FloodFillStrategy::think(const WallInfo& walls, int x, int y, int heading) {
    // 1. 到达终点则停止
    if ((x == 7 || x == 8) && (y == 7 || y == 8)) {
        return RobotAction::STOP_FINISH;
    }

    // 2. 更新本格墙壁信息，重新灌水
    updateWalls(x, y, heading, walls);
    floodFill();

    // 3. 在可通行的邻居中找距离最小的格子
    const int dx[4] = {0, 1,  0, -1};
    const int dy[4] = {1, 0, -1,  0};

    int bestDir  = -1;
    uint8_t bestDist = INF;

    for (int dir = 0; dir < 4; ++dir) {
        // 检查该方向是否有墙
        bool blocked = false;
        switch (dir) {
            case 0: blocked = hasNorthWall(x, y); break;
            case 1: blocked = hasEastWall (x, y); break;
            case 2: blocked = hasSouthWall(x, y); break;
            case 3: blocked = hasWestWall (x, y); break;
        }
        if (blocked) continue;

        int nx = x + dx[dir];
        int ny = y + dy[dir];
        if (nx < 0 || nx >= SIZE || ny < 0 || ny >= SIZE) continue;

        if (distances[ny][nx] < bestDist) {
            bestDist = distances[ny][nx];
            bestDir  = dir;
        }
    }

    // 4. 无路可走（理论上不应发生）
    if (bestDir == -1) return RobotAction::STOP_FINISH;

    // 5. 将绝对方向转换为相对动作
    return dirToAction(bestDir, heading);
}