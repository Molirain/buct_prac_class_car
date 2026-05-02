#pragma once

#include "maze_strategy.h"
#include <cstdint>

class FloodFillStrategy : public MazeStrategy {
public:
    FloodFillStrategy();

    RobotAction think(const WallInfo& walls, int x, int y, int heading) override;
    void reset() override;

private:
    static constexpr int SIZE = 16;
    static constexpr uint8_t INF = 255;

    // 距离图
    uint8_t distances[SIZE][SIZE];

    // 水平墙：hWalls[y][x] = true 表示格子(x,y)的 北侧 有墙
    // y 范围 0..16，共17条水平线
    bool hWalls[SIZE + 1][SIZE];

    // 垂直墙：vWalls[y][x] = true 表示格子(x,y)的 西侧 有墙
    // x 范围 0..16，共17条垂直线
    bool vWalls[SIZE][SIZE + 1];

    // 环形队列（BFS用，无动态分配）
    struct Queue {
        static constexpr int CAP = SIZE * SIZE;
        uint8_t xs[CAP];
        uint8_t ys[CAP];
        int head, tail, count;

        void clear()          { head = tail = count = 0; }
        bool empty()    const { return count == 0; }
        void push(int x, int y) {
            xs[tail] = static_cast<uint8_t>(x);
            ys[tail] = static_cast<uint8_t>(y);
            tail = (tail + 1) % CAP;
            ++count;
        }
        void pop(int& x, int& y) {
            x = xs[head]; y = ys[head];
            head = (head + 1) % CAP;
            --count;
        }
    } queue;

    // 墙壁查询（统一接口）
    bool hasNorthWall(int x, int y) const { return hWalls[y + 1][x]; }
    bool hasSouthWall(int x, int y) const { return hWalls[y][x];     }
    bool hasEastWall (int x, int y) const { return vWalls[y][x + 1]; }
    bool hasWestWall (int x, int y) const { return vWalls[y][x];     }

    void initBoundaryWalls();
    void updateWalls(int x, int y, int heading, const WallInfo& walls);
    void floodFill();

    // heading: 0北 1东 2南 3西
    // 返回从 heading 朝向看，传感器left/front/right对应的绝对方向
    static int toAbsolute(int heading, int relativeDir);
    // 根据目标方向和当前朝向，返回动作
    static RobotAction dirToAction(int targetAbsDir, int heading);
};