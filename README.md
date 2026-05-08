BUCT 迷宫小车项目 (Maze Solver Robot)
=====================================

禁止 vibe coding！

概述 (OVERVIEW)
---------------
本项目为一个基于 STM32H750VB 微控制器的智能迷宫求解小车源代码库。该系统通过多传感器融合与路径规划算法（如泛滥填充、右手法则），在未知迷宫环境中实现自主探索与导航。系统底层依赖 STM32 HAL 库，并采用 FreeRTOS 进行多任务实时调度。

系统架构 (SYSTEM ARCHITECTURE)
------------------------------
项目构建基于 PlatformIO 平台，底层外设初始化由 STM32CubeMX 生成。代码逻辑采用高度模块化的设计：

核心目录结构：
  * src/app/      - FreeRTOS 顶层应用任务（如控制、通讯、电机、传感器模块调度）。
  * src/driver/   - 硬件外设驱动层（MPU6050、直流电机驱动、霍尔编码器、超声波测距底层读取及里程计）。
  * src/module/   - 高级算法与功能模块（底盘运动学控制、泛滥填充策略、右手法则策略、传感器数据滤波）。
  * lib/          - 第三方与独立算法库（包含 QuickPID 控制库）。

硬件规格 (HARDWARE SPECIFICATION)
---------------------------------
* 微控制器：       STM32H750VB (ARM Cortex-M7)
* 电源系统：       3S 航模锂电池，配合外部降压稳压模块
* 姿态感知：       MPU6050 运动处理组件（航向角解算与反馈）
* 里程与速度采集： 霍尔编码器
* 测距与环境感知： 超声波传感器模块（避障及迷宫墙壁距离检测）
* 执行机构：       直流减速电机 + 电机驱动板
* 通讯接口：       蓝牙透传模块（参数整定、遥测与指令下发）

任务子系统 (SUBSYSTEM TASKS)
----------------------------
系统基于 FreeRTOS 划分为四大独立且高内聚的线程任务：

1. task_motor  (电机控制任务)
   负责底层电机 PWM 占空比的周期性更新。依据编码器反馈的数据，执行严格的速度与位置闭环 PID 控制。

2. task_sensor (传感器采集任务)
   以固定周期读取超声波测距数据并进行滤波（sensor_filter）。负责读取并解算 MPU6050 姿态角信息，结合里程计（DiffDriveOdometry）更新小车的全局坐标位姿。

3. task_ctrl   (主控与路径任务)
   作为小车的核心逻辑任务运转。执行迷宫求解策略计算（flood_fill_strategy, right_hand_strategy），并根据传感器环境特征，向底层底盘控制器发布对应的位移和转向指令。

4. task_comm   (通讯与交互任务)
   处理通过蓝牙与上位机的数据收发链路。解析遥控指令，同时实时回传小车状态数据（内部位姿参数、PID 数据等状态机变量）。

构建与烧录 (BUILDING AND FLASHING)
----------------------------------
环境依赖：
推荐使用 Visual Studio Code 配合 PlatformIO IDE 插件完成本项目的构建。

代码编译：
    $ git clone https://github.com/Molirain/buct_prac_class_car.git
    $ cd buct_prac_class_car
    $ pio run

固件烧录 (DFU 模式)：
本项目 platformio.ini 默认采用 dfu-util 方式烧录。确保芯片处于 DFU 引导模式：
1. 短接 BOOT0 引脚，复位微控制器进入 DFU 模式。
2. 连接 USB 数据线。
3. 执行以下命令完成烧录：
    $ pio run -e genericSTM32H750VB -t upload

许可证 (LICENSE)
----------------
关于本项目的开源协议及底层驱动库授权详情，请参阅源代码包内的 LICENSE.txt 文件。
