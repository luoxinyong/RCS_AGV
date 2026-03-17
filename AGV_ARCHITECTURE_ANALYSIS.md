# AGV 软件架构分析文档

> 工作空间: `/home/bnxy/SLAM/test_ws`
> 分析日期: 2026-03-16
> 目的: 为后续 AI 辅助架构重设计提供完整的现有系统理解

---

## 目录

- [1. 系统概述](#1-系统概述)
- [2. 功能包清单与职责](#2-功能包清单与职责)
- [3. task_manager 核心架构](#3-task_manager-核心架构)
  - [3.1 初始化阶段：跨包连接建立](#31-初始化阶段跨包连接建立)
  - [3.2 主循环 RunCycle()：状态机](#32-主循环-runcycle状态机)
  - [3.3 ExecuteTask()：任务分发中枢](#33-executetask任务分发中枢)
  - [3.4 路径跟踪与避障：GenerateAndPubFollowPathData()](#34-路径跟踪与避障generateandpubfollowpathdata)
  - [3.5 货叉控制：GenerateAndPubWorkData()](#35-货叉控制generateandpubworkdata)
  - [3.6 取货联动：GenerateAndPubPickCargoData()](#36-取货联动generateandpubpickcargodatarfid--相机--plc)
  - [3.7 充电控制：GenerateAndPubChargeData()](#37-充电控制generateandpubchargedata)
  - [3.8 原地自旋：GenerateAndPubAroundSpinData()](#38-原地自旋generateandpubaroundspindata)
  - [3.9 子任务完成判定：FinishedCurrentStepTask()](#39-子任务完成判定finishedcurrentsteptask)
  - [3.10 状态上报：PubIpcResData()](#310-状态上报pubipcresdata)
- [4. 跨包数据流全景图](#4-跨包数据流全景图)
- [5. 各功能包详细说明](#5-各功能包详细说明)
  - [5.1 3d_localization_csg（loc_ad）](#51-3d_localization_csgloc_ad)
  - [5.2 common_msgs](#52-common_msgs)
  - [5.3 costmap_2d](#53-costmap_2d)
  - [5.4 global_planner](#54-global_planner)
  - [5.5 nav_core](#55-nav_core)
  - [5.6 navfn](#56-navfn)
  - [5.7 mpc_path_follower](#57-mpc_path_follower)
  - [5.8 plc_interaction](#58-plc_interaction)
  - [5.9 rcs_interaction](#59-rcs_interaction)
  - [5.10 hipnuc_imu](#510-hipnuc_imu)
  - [5.11 rslidar_sdk](#511-rslidar_sdk)
  - [5.12 vanjee_lidar_sdk](#512-vanjee_lidar_sdk)
- [6. 关键跨包调用汇总表](#6-关键跨包调用汇总表)
- [7. 架构特点与重设计建议方向](#7-架构特点与重设计建议方向)

---

## 1. 系统概述

本系统是一套完整的 **AGV（自动导引车）软件栈**，基于 ROS1（Noetic）构建，覆盖从硬件驱动到调度通信的完整链路。系统核心是 `task_manager` 包，它充当所有子系统的"大脑"——从 RCS 接收任务、协调导航规划、驱动底盘和执行器、最终反馈执行状态。

### 整体架构分层

```
┌─────────────────────────────────────────────────────────────────┐
│                    调度层 (RCS 外部系统)                         │
│                    Modbus TCP ↕ rcs_interaction                 │
├─────────────────────────────────────────────────────────────────┤
│                    决策层 (task_manager)                         │
│              状态机 + 路径跟踪 + 避障 + 任务分发                  │
├────────────────┬──────────────────┬─────────────────────────────┤
│   规划层        │   感知/定位层      │   通信层                    │
│ global_planner │ 3d_localization   │ plc_interaction             │
│ navfn          │ rslidar_sdk       │ (自定义 UDP)                │
│ costmap_2d     │ vanjee_lidar_sdk  │                             │
│ mpc_follower   │ hipnuc_imu        │                             │
├────────────────┴──────────────────┴─────────────────────────────┤
│                    硬件层                                        │
│   PLC底盘 │ LiDAR × 2 │ IMU │ RFID │ M4 ToF相机 │ 充电桩       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 功能包清单与职责

| 序号 | 功能包 | 一句话职责 | 角色 |
|------|--------|-----------|------|
| 1 | `task_manager` | AGV 任务调度中枢，协调所有子系统 | **核心** |
| 2 | `rcs_interaction` | RCS 调度系统 Modbus TCP 通信桥梁 | 上行通信 |
| 3 | `plc_interaction` | PLC 底盘/执行器 UDP 通信桥梁 | 下行通信 |
| 4 | `common_msgs` | 定义系统内所有自定义消息格式（9 个 msg） | 消息定义 |
| 5 | `3d_localization_csg` | 3D LiDAR + IMU 融合定位 | 定位 |
| 6 | `costmap_2d` | 2D 代价地图（含自定义 FreeLayerBounds 层） | 环境感知 |
| 7 | `global_planner` | 全局路径规划（Dijkstra/A*/Frenet + B-spline 优化） | 路径规划 |
| 8 | `navfn` | Dijkstra/A* 全局规划器（备选） | 路径规划 |
| 9 | `nav_core` | 导航接口抽象层（BaseGlobalPlanner/BaseLocalPlanner） | 接口定义 |
| 10 | `mpc_path_follower` | MPC 局部路径跟踪器（实验性，当前注释） | 局部规划 |
| 11 | `rslidar_sdk` | RoboSense LiDAR 驱动（RSHELIOS/16P） | 硬件驱动 |
| 12 | `vanjee_lidar_sdk` | 万集 LiDAR 驱动（14 种型号） | 硬件驱动 |
| 13 | `hipnuc_imu` | HiPNUC IMU 串口驱动 | 硬件驱动 |

---

## 3. task_manager 核心架构

> 源码位置: `src/task_manager/src/task_manager.cpp`（2744 行）
> 头文件: `src/task_manager/include/task_manager.h`（845 行）
> 节点入口: `src/task_manager/src/task_manager_node.cpp`
> 运行频率: **100 Hz**

### 3.1 初始化阶段：跨包连接建立

构造函数 `TaskManager::TaskManager()` 是整个系统的"接线板"，在此建立与所有外部包的连接。

#### 3.1.1 创建全局代价地图 → `costmap_2d` 包

```cpp
// task_manager.cpp:112-125
planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
planner_costmap_ros_->pause();
// ... 加载规划器后 ...
planner_costmap_ros_->start();
```

- **调用目标**: `costmap_2d` 包的 `Costmap2DROS` 类
- **实现功能**: 创建并启动全局代价地图实例。该地图持续从 LiDAR 传感器数据更新障碍物信息，维护一个包含 StaticLayer + ObstacleLayer + InflationLayer + FreeLayerBounds（自定义）的分层代价地图
- **被谁使用**: 后续的 `obstacleDetection()` 和 `GenerateGlobalPath()` 都依赖此地图

#### 3.1.2 动态加载全局路径规划器 → `nav_core` + `global_planner`/`navfn` 包

```cpp
// task_manager.cpp:53 (参数)
nh_.param<std::string>("/base_global_planner", global_planner_, std::string("navfn/NavfnROS"));

// task_manager.cpp:115-124 (加载)
planner_ = bgp_loader_.createInstance(global_planner_);
planner_->initialize(bgp_loader_.getName(global_planner_), planner_costmap_ros_);
```

- **调用目标**: `nav_core::BaseGlobalPlanner` 接口（由 `nav_core` 包定义），通过 `pluginlib` 动态加载具体实现
- **默认加载**: `navfn/NavfnROS`（Dijkstra 算法）；可通过参数切换为 `global_planner/GlobalPlanner`（支持 Dijkstra/A*/Frenet + B-spline 优化）
- **实现功能**: 为避障时的路径重规划提供全局搜索能力。规划器在 `costmap_2d` 地图上搜索从当前位置到目标点的无碰撞路径

#### 3.1.3 订阅 RCS 任务 → `rcs_interaction` 包

```cpp
// task_manager.cpp:76
sub_rcs_task_ = nh_.subscribe(topic_task_rcs_, 10, &TaskManager::TaskProcess, this);
```

- **消息来源**: `rcs_interaction` 包发布的 `task_rcs` 话题
- **消息类型**: `common_msgs::rcs_cmd_data`（定义在 `common_msgs` 包中）
- **实现功能**: 接收 RCS 调度系统下发的任务指令。`rcs_interaction` 作为 Modbus TCP 服务端，从 RCS 的保持寄存器中读取二进制任务数据，封装为 ROS 消息发布
- **数据内容**: 任务命令类型（接收/执行/暂停/终止）、任务元数据、N 个子任务的二进制编码数据（每个子任务 20 字节）

#### 3.1.4 订阅 PLC 反馈 → `plc_interaction` 包

```cpp
// task_manager.cpp:78-79
sub_plc_res_        = nh_.subscribe("plc_res_nav_data", 10, &TaskManager::ProcessPlcResData, this);
sub_plc_action_res_ = nh_.subscribe("plc_res_action_data", 10, &TaskManager::ProcessPlcResActionData, this);
```

- **消息来源**: `plc_interaction` 包发布的两个反馈话题
- **消息类型**: `common_msgs::plc_res_nav_data` + `common_msgs::plc_res_action_data`
- **实现功能**: 接收 PLC 底盘控制器的实时反馈
  - **导航反馈** (`ProcessPlcResData`, task_manager.cpp:2121-2130): 电池容量、线速度、传感器到位信号、停车到位状态、充电状态、到站节点 ID、当前楼层号
  - **执行器反馈** (`ProcessPlcResActionData`, task_manager.cpp:2132-2141): 执行器状态（空闲/上升到位/下降到位/执行中）、故障码、实际动作值（如叉齿高度）、已完成动作的序列号

#### 3.1.5 订阅定位数据 → `3d_localization_csg` 包

```cpp
// task_manager.cpp:69-74
if (flag_use_2d_loc_) {
    sub_current_pose_ = nh_.subscribe("/tracked_pose", 100, &TaskManager::Current2dPoseProcess, this);
} else {
    sub_current_pose_ = nh_.subscribe("/fused_localization", 100, &TaskManager::Current3dPoseProcess, this);
}
```

- **消息来源**: `3d_localization_csg`（loc_ad）包的 `filtering_node` 发布的 `/fused_localization`
- **消息类型**: `nav_msgs::Odometry`
- **实现功能**: 获取 AGV 的实时位姿（x, y, z, quaternion）和速度。回调函数 `Current3dPoseProcess()`（task_manager.cpp:2501-2520）将数据存储到 `pose_current_`、`yaw_current_`、`twist_current_` 等成员变量，**这些变量是所有路径跟踪、偏差计算、状态上报的基础数据源**

#### 3.1.6 初始化 RFID 读写器

```cpp
// task_manager.cpp:129
flag_rfid_enabled_ = rfid_.init();
```

- **调用目标**: 内部 `RFIDReader` 类 → RS232 串口（/dev/ttyUSB0, 115200bps）→ RFID 硬件
- **实现功能**: 初始化 RFID 读卡器连接，用于取货任务完成后验证货物 EPC 标签是否与 RCS 下发的预期值匹配

#### 3.1.7 初始化托盘识别相机（可选）

```cpp
// task_manager.cpp:83-96
if (flag_enable_pallet_identify_) {
    p_pallet_identify_ = std::make_unique<pallet_idenfity::PalletIdentifyM4>();
    flag_init_camera_ = p_pallet_identify_->InitPalletIdentify(ip_camera_dtof_);
}
```

- **调用目标**: 内部 `PalletIdentifyM4` 类 → TCP Socket（端口 5501）→ M4 ToF 相机
- **实现功能**: 连接深度相机，用于取货任务中实时获取托盘的精确位姿（x, y, theta）

---

### 3.2 主循环 RunCycle()：状态机

> 源码位置: task_manager.cpp:1948-2033

`RunCycle()` 每 10ms（100Hz）执行一次，是整个 AGV 系统的心跳。它实现了一个 **8 状态有限状态机**：

```
STATUS_NONE（初始化）
    │
    ▼
STATUS_NO_TASK（空闲等待）
    │  ← 收到 TASK_CMD_RECV
    ▼
STATUS_RECVING_TASK（任务已接收，路径校验完成）
    │  ← 收到 TASK_CMD_EXECUTE
    ▼
STATUS_EXECUTING_TASK（执行中）
    ├──→ 收到 TASK_CMD_PAUSE → STATUS_PAUSE_TASK_BY_RCS
    └──→ 所有子任务完成 → STATUS_COMPLETED_TASK → 自动回到 STATUS_NO_TASK

STATUS_PAUSE_TASK_BY_RCS
    └──→ 收到 TASK_CMD_OBORT → STATUS_ABORT_TASK → 自动回到 STATUS_NO_TASK
```

#### 每个周期的固定动作

```
RunCycle() 每次调用:
  ├── PubFloorNum()      → 定期发布楼层号到 3d_localization_csg
  ├── PubIpcResData()    → 每周期发布 AGV 完整状态到 rcs_interaction → RCS
  └── switch(status_ipc_) → 根据状态执行对应逻辑
```

#### STATUS_NO_TASK + TASK_CMD_RECV 时的处理

```cpp
// task_manager.cpp:1958-1977
vec_step_task_stru_global_.assign(vec_step_task_stru_.begin(), vec_step_task_stru_.end());
CheckRcsPathTaskValidity(vec_step_task_stru_global_);  // 校验路径连续性
GenerateGlobalDensePath(vec_step_task_stru_global_, vec_path_point_global_);  // 生成稠密路径
ShowRcsTaskPoints(vec_step_task_stru_global_);  // RViz 显示任务点
ShowGlobalDensePath(vec_path_point_global_);    // RViz 显示稠密路径
```

- `CheckRcsPathTaskValidity()`（task_manager.cpp:535-554）: 遍历所有子任务，检查每个子任务的终点是否等于下一个子任务的起点
- `GenerateGlobalDensePath()`（task_manager.cpp:630-698）: 按子任务类型分别调用 `GenerateLinePoints()`（直线插值，步长 0.1m）和 `GenerateArcPoints()`（弧线插值）生成稠密路径点序列

---

### 3.3 ExecuteTask()：任务分发中枢

> 源码位置: task_manager.cpp:1864-1932

当状态为 `STATUS_EXECUTING_TASK` 时，每个 100Hz 周期都调用此函数。它根据当前子任务的 `type` 字段分发到不同的处理函数：

```
ExecuteTask()
│
├── type=1 (RCS_NODE_LINE)           → task_type_curr_ = IPC_FOLLOW_PATH
│   └── GenerateAndPubFollowPathData()    ──→ plc_interaction
│
├── type=2 (RCS_NODE_CURVE)
│   ├── map_id=1 (原地自旋)          → task_type_curr_ = IPC_AROUND_SPIN
│   │   └── GenerateAndPubAroundSpinData() ──→ plc_interaction
│   └── map_id=0 (弧线行驶)          → task_type_curr_ = IPC_FOLLOW_PATH
│       └── GenerateAndPubFollowPathData() ──→ plc_interaction
│
├── type=3 (RCS_NODE_WORK)           → task_type_curr_ = IPC_WORK
│   └── GenerateAndPubWorkData()          ──→ plc_interaction
│
├── type=11 (RCS_NODE_AUTORUN_BY_CAMERA) → task_type_curr_ = IPC_PICK_CARGO
│   └── GenerateAndPubPickCargoData()     ──→ plc_interaction + RFID + M4相机
│
├── type=4 (RCS_NODE_CHARGE)         → task_type_curr_ = IPC_CHARGE
│   └── GenerateAndPubChargeData()        ──→ plc_interaction
│
├── type=99 (RCS_NODE_TASK_FINISH)   → task_type_curr_ = IPC_TASK_FINISH
│   └── GenerateAndPubNoTaskData()        ──→ plc_interaction
│
└── UpdataCurrentStepTaskIndex()     → 检查当前子任务是否完成 → 自动切换到下一个
```

每个 `GenerateAndPub*()` 函数最终都通过 **两个话题** 将数据发送到 `plc_interaction`：
- `pub_nav_data_.publish(data_nav)` → `nav_data` 话题 → plc_interaction → UDP → PLC 底盘导航控制
- `pub_action_data_.publish(data_action)` → `action_data` 话题 → plc_interaction → UDP → PLC 执行器控制

---

### 3.4 路径跟踪与避障：GenerateAndPubFollowPathData()

> 源码位置: task_manager.cpp:1316-1510
> **这是整个系统中最复杂、跨包调用最密集的函数**

#### 3.4.1 避障状态机（4 阶段）

该函数内部维护了一个独立的 **避障状态机**，在路径跟踪过程中实时检测和应对障碍物：

```
FOLLOW_ORIGINAL（跟踪原始路径）
    │  连续 5 帧检测到障碍 && obs_area 第 14 位允许避障
    ▼
AVOIDANCE_PLANNING（规划绕障路径）
    │  GenerateGlobalPath() 规划成功
    ▼
FOLLOW_AVOIDANCE（跟踪绕障路径）
    ├──→ 连续 3 帧又检测到障碍 → 回到 AVOIDANCE_PLANNING（重新规划）
    └──→ isOvercomeObstacle() 连续 2 帧确认绕过障碍 → RETURN_TO_ORIGINAL
          │
          ▼
RETURN_TO_ORIGINAL（回归原始路径）
    ├──→ 合并路径无障碍 → 回到 FOLLOW_ORIGINAL
    └──→ 合并路径有障碍 → 回到 FOLLOW_AVOIDANCE

特殊规则: 任何状态下，剩余距离 ≤ 7m → 强制回到 FOLLOW_ORIGINAL（确保到达终点）
```

#### 3.4.2 障碍检测 → 调用 `costmap_2d` 包

```cpp
// task_manager.cpp:1351
bool origin_obs_now = obstacleDetection(original_path, pose_current_, obs_points);
```

`obstacleDetection()` 函数（task_manager.cpp:2663-2731）的跨包调用链：

| 步骤 | 调用 | 来源包 | 作用 |
|------|------|--------|------|
| 1 | `planner_costmap_ros_->getCostmap()` | `costmap_2d` | 获取 2D 代价地图指针 |
| 2 | `planner_costmap_ros_->getRobotFootprint()` | `costmap_2d` | 获取机器人多边形足迹 |
| 3 | `costmap_2d::transformFootprint(x, y, theta, ...)` | `costmap_2d` | 将足迹变换到路径各点位姿 |
| 4 | `costmap->convexFillCells(map_footprint, cells)` | `costmap_2d` | 填充足迹覆盖的栅格单元 |
| 5 | `costmap->getCost(cell.x, cell.y)` | `costmap_2d` | 逐单元检测代价值 |
| 6 | 判断 `cost >= INSCRIBED_INFLATED_OBSTACLE` | `costmap_2d` | 确认是否为障碍 |

**工作原理**: 从最近路径点开始，沿路径前方每个点都将机器人足迹投影到代价地图上，检测足迹覆盖区域内是否存在致命障碍（代价值 ≥ 253）。

#### 3.4.3 绕障规划 → 调用 `global_planner`/`navfn` 包

```cpp
// task_manager.cpp:1371
if (!GenerateGlobalPath(goal, path_global_))
```

`GenerateGlobalPath()` 函数（task_manager.cpp:146-167）的跨包调用链：

| 步骤 | 调用 | 来源包 | 作用 |
|------|------|--------|------|
| 1 | `planner_costmap_ros_->getCostmap()->getMutex()` | `costmap_2d` | 获取线程互斥锁 |
| 2 | `planner_->makePlan(start, goal, plan)` | `nav_core` 接口 → `navfn` 或 `global_planner` 实现 | 在代价地图上搜索从当前位置到终点的无碰撞路径 |

如果加载的是 `navfn/NavfnROS`:
- 在 `costmap_2d` 地图上执行 Dijkstra 波前传播
- 通过梯度下降提取路径（亚像素精度）

如果加载的是 `global_planner/GlobalPlanner`:
- 支持 Dijkstra / A* / Frenet 规划器
- 可选 B-spline 轨迹优化和安全走廊约束
- 可选曲率优化减少尖锐转弯

#### 3.4.4 偏差计算与数据发布 → `plc_interaction` 包

路径跟踪的核心计算流程：

```
1. 计算路径偏差:
   ├── 直线: CalculateLinePathDeviation()  → delta_x（纵向）, delta_y（横向）, delta_theta（航向）
   └── 弧线: CalculateCurvePathDeviation() → 同上 + 车头/车尾投影

2. 计算前瞻点:
   └── GetLookaheadPoint() → 沿路径累积距离 ≥ distance_lookahead_（动态，随速度增大）

3. 脱轨检测:
   └── |delta_y| > dis_derail_(0.2m) → SetErrorCode(ERROR_NAVI_DERAILMENT)

4. 组装并发布 nav_data:
   └── pub_nav_data_.publish(data_nav) → plc_interaction 订阅 → UDP → PLC
```

**发布到 PLC 的关键字段**:

| 字段 | 含义 | 作用 |
|------|------|------|
| `point_lookahead` | 前瞻点坐标 | PLC 底盘控制器的跟踪目标 |
| `delta_center` | 车体中心横向偏差 | PLC 用于纠偏控制 |
| `delta_head` / `delta_tail` | 车头/车尾横向偏差 | PLC 用于大车身纠偏 |
| `delta_angle` | 航向角偏差（度） | PLC 用于转向控制 |
| `dis_remain_curr_node` | 当前子任务剩余距离 | PLC 用于减速规划 |
| `dis_to_stop_node` | 到停车点距离 | PLC 用于提前制动 |
| `flag_need_avoid` | 是否在走绕障路径 | PLC 调整跟踪策略 |
| `obs_area` | 避障区域标志位 | PLC 选择传感器方案 |

---

### 3.5 货叉控制：GenerateAndPubWorkData()

> 源码位置: task_manager.cpp:1754-1781

```cpp
pub_nav_data_.publish(data_nav);   // 导航参数全部清零（停车状态）
PubActionData(ACTUATOR_CMD_FORK_DOWN, static_work, id_action_current_);
// → pub_action_data_.publish() → plc_interaction → UDP → PLC 执行器
```

- **跨包调用**: 通过 `action_data` 话题 → `plc_interaction` 包 → UDP 发送到 PLC
- **PLC 行为**: 收到 `ACTUATOR_CMD_FORK_DOWN` + `static_work` 值后，PLC 根据叉齿实时高度自行判断是上升还是下降
- **完成判定**: PLC 完成后上报 `plc_res_action_data`（status=上升到位/下降到位, id=当前动作序列号），task_manager 通过 `g_id_finish_action_1 == id_action_current_` 确认

---

### 3.6 取货联动：GenerateAndPubPickCargoData()（RFID + 相机 + PLC）

> 源码位置: task_manager.cpp:1784-1836
> **这是跨系统交互最复杂的函数，涉及 3 个独立硬件系统**

#### 执行阶段（每 100Hz 调用一次）

```
GenerateAndPubPickCargoData()
│
├── [1] 启动 RFID 异步扫描
│   rfid_.startScan()  → RS232 → RFID 硬件（后台线程持续扫描）
│
├── [2] 获取托盘位姿（M4 ToF 相机）
│   p_pallet_identify_->GetPalletIdentifyData()  → TCP:5501 → M4 相机
│   → 写入 data_nav.x_detect / y_detect / theta_detect
│
├── [3] 发布导航数据到 PLC
│   pub_nav_data_.publish(data_nav)
│   关键字段: flag_pick_task_valid=1, flag_detect_pallet, 相机坐标
│   → plc_interaction → UDP → PLC → 底盘执行精确对位
│
└── [4] 发布执行器停止指令
    PubActionData(ACTUATOR_CMD_STOP, 0, 0) → plc_interaction → PLC
```

#### 完成阶段（在 FinishedCurrentStepTask() 中, task_manager.cpp:339-383）

```
当 g_status_stop_==2 (PLC 上报停车到位):
│
├── rfid_.stopScan()          ← 停止 RFID 扫描
├── rfid_.getResult()         ← 获取扫描到的 EPC 列表
├── 还原预期 EPC 字符串      ← 从 RCS 下发的 expected_epc_high/low
├── 比对实际 EPC vs 预期 EPC
│   ├── 匹配  → data_res_ipc_.rfid_result = 1
│   ├── 不匹配 → data_res_ipc_.rfid_result = 2
│   └── 未扫到 → data_res_ipc_.rfid_result = 3
│
└── 结果通过 PubIpcResData() → rcs_interaction → Modbus → RCS 系统
```

---

### 3.7 充电控制：GenerateAndPubChargeData()

> 源码位置: task_manager.cpp:1271-1301

```cpp
data_nav.flag_charge = charge_mode;  // 0=未知, 1=充电, 2=停止充电
pub_nav_data_.publish(data_nav);     // → plc_interaction → UDP → PLC → 充电桩
```

- **完成判定**: `g_status_charge_ == 2`（PLC 通过 `plc_res_nav_data` 反馈充电结束）

---

### 3.8 原地自旋：GenerateAndPubAroundSpinData()

> 源码位置: task_manager.cpp:1678-1751

```
初始化:
├── 记录起始 yaw: spin_start_yaw_
├── 根据 heading 编码解析转向方向: flag_direction_ (1=左旋, -1=右旋, 0=自动)
└── 计算目标旋转角: spin_total_angle_

每周期:
├── 累积已旋转角度 (unwrap 处理防跳变)
├── 计算剩余角度: remaining = spin_total_angle_ - rotated
├── data_nav.angle_diff_spin_around = remaining
└── pub_nav_data_.publish() → plc_interaction → PLC 转向电机
```

- **完成判定**: `g_id_stop_node_ == id_a && g_status_stop_ == 2`（PLC 确认自旋完成）

---

### 3.9 子任务完成判定：FinishedCurrentStepTask()

> 源码位置: task_manager.cpp:257-409

每种任务类型的完成条件都依赖不同外部系统的反馈：

| 任务类型 | 完成条件 | 数据来源 |
|---------|---------|---------|
| `IPC_FOLLOW_PATH` | 剩余距离 < 阈值（连续行驶 0.5m / 到站 0.03m）+ 速度 < 0.001 m/s | `3d_localization_csg`(位姿计算) + `plc_interaction`(速度反馈) |
| `IPC_AROUND_SPIN` | PLC 上报 `g_id_stop_node_==id_a` 且 `g_status_stop_==2` | `plc_interaction`(到站状态) |
| `IPC_PICK_CARGO` | PLC 上报停车到位 + RFID EPC 比对 | `plc_interaction` + RFID 硬件 |
| `IPC_WORK` | 货叉动作完成（status=上升到位/下降到位）且动作 ID 匹配 | `plc_interaction`(plc_res_action_data) |
| `IPC_CHARGE` | `g_status_charge_==2` | `plc_interaction`(充电状态) |
| `IPC_TASK_FINISH` | 立即完成 | 无外部依赖 |

完成后由 `UpdataCurrentStepTaskIndex()`（task_manager.cpp:411-438）自动推进到下一个子任务，直到所有子任务完成则 `status_ipc_ = STATUS_COMPLETED_TASK`。

---

### 3.10 状态上报：PubIpcResData()

> 源码位置: task_manager.cpp:440-532
> **每个 100Hz 周期都调用，是 AGV 对外的"心跳报文"**

```
数据组装:
├── 任务元数据: tr_cmd, tr_id_*, tr_step_size, tr_complete_percent
├── 当前子任务信息: node_type, node_next_type, node_info_1~20 (20 个寄存器)
├── AGV 位姿: agv_pose_x(mm), agv_pose_y(mm), agv_pose_theta(0.1°)
├── 运动状态: agv_vel(mm/s), lift_height, steering_angle
├── 系统状态: battery_soc, status_charge, task_status, mode_status
├── 故障信息: error_1 (位掩码)
├── 货物验证: cargo_loaded, actual_epc_high/low, rfid_result
└── 发布: pub_ipc_res_data_.publish(res_data)

数据流:
task_manager → ipc_res_data 话题 → rcs_interaction 订阅
    → 写入 Modbus 输入寄存器（~250 个 uint16）
    → RCS 调度系统通过 Modbus 读取
```

---

## 4. 跨包数据流全景图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           RCS 调度系统（外部）                            │
└───────────┬──────────────────────────────────────────────┬───────────────┘
            │ Modbus TCP (保持寄存器: 任务指令)              ▲ Modbus TCP (输入寄存器: AGV 状态)
            ▼                                              │
┌───────────────────────┐                     ┌────────────────────────┐
│   rcs_interaction     │                     │   rcs_interaction      │
│   ReadHoldingRegs()   │                     │   WriteInputRegs()     │
│   → task_rcs 话题     │                     │   ← ipc_res_data 话题  │
└───────────┬───────────┘                     └────────────┬───────────┘
            │ common_msgs::rcs_cmd_data                    ▲ common_msgs::ipc_res_data
            ▼                                              │
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│                          task_manager (核心)                              │
│                                                                          │
│  TaskProcess()     → 解析二进制任务                                       │
│  RunCycle()        → 状态机驱动                                           │
│  ExecuteTask()     → 子任务分发                                           │
│  PubIpcResData()   → 状态上报                                            │
│                                                                          │
└──────┬───────────────────┬───────────────┬──────────────────┬────────────┘
       │                   │               │                  │
       │ nav_data          │ action_data   │ (内部)            │ (内部)
       │                   │               │                  │
       ▼                   ▼               ▼                  ▼
┌──────────────┐  ┌──────────────┐  ┌───────────┐     ┌────────────┐
│plc_interaction│  │plc_interaction│  │  RFID     │     │ M4 相机     │
│ NavDataProc() │  │ActionDataProc│  │ RS232     │     │ TCP:5501   │
│ → UDP:1101    │  │ → UDP:1103   │  │ startScan │     │ GetPallet  │
└──────┬────────┘  └──────┬───────┘  │ stopScan  │     │ Identify   │
       │                  │          │ getResult  │     │ Data       │
       ▼                  ▼          └───────────┘     └────────────┘
┌─────────────────────────────┐
│         PLC 底盘控制器        │
│  导航控制 │ 执行器控制        │
│  (电机/转向/制动) (货叉/滚筒) │
└──────────┬──────────────────┘
           │ plc_res_nav_data + plc_res_action_data
           │ (UDP → plc_interaction → ROS 话题)
           ▼
    task_manager 订阅反馈
    (ProcessPlcResData / ProcessPlcResActionData)


  感知/定位子系统（独立运行，为 task_manager 提供数据）:

  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
  │ rslidar_sdk  │     │vanjee_lidar  │     │ hipnuc_imu   │
  │ /rslidar_pts │     │/vanjee_pts   │     │ /IMU_data    │
  └──────┬───────┘     └──────┬───────┘     └──────┬───────┘
         │ PointCloud2        │ PointCloud2        │ Imu
         └─────────┬──────────┘                    │
                   ▼                               │
         ┌─────────────────────────┐               │
         │  3d_localization_csg    │◄──────────────┘
         │  data_pretreat_node     │
         │  filtering_node         │
         │  (ESKF + Small-GICP)    │
         └────────┬────────────────┘
                  │ /fused_localization (100Hz)
                  ▼
           task_manager.Current3dPoseProcess()
           → pose_current_, yaw_current_

         ┌─────────────────────────┐
         │      costmap_2d         │◄── LiDAR 点云自动更新
         │  (StaticLayer +         │
         │   ObstacleLayer +       │
         │   InflationLayer +      │
         │   FreeLayerBounds)      │
         └────────┬────────────────┘
                  │ getCost() / transformFootprint() / convexFillCells()
                  ▼
           task_manager.obstacleDetection()
           task_manager.GenerateGlobalPath() → planner_->makePlan()
                                                 │
                                    ┌────────────┴────────────┐
                                    ▼                         ▼
                              navfn/NavfnROS          global_planner/
                              (Dijkstra/A*)           GlobalPlanner
                                                      (Dijkstra/A*/
                                                       Frenet/B-spline)
```

---

## 5. 各功能包详细说明

### 5.1 3d_localization_csg（loc_ad）

| 项目 | 详情 |
|------|------|
| **职责** | 基于预建地图的 3D LiDAR 融合定位 |
| **核心算法** | Error-State Kalman Filter（15 维状态）+ Small-GICP 点云配准 |
| **状态维度** | 位置(3) + 速度(3) + 姿态(3) + 加速度计偏置(3) + 陀螺仪偏置(3) |
| **节点** | `data_pretreat_node`（传感器同步与去畸变）+ `filtering_node`（定位融合） |
| **输入话题** | `/velodyne_points`, `/imu_raw`, `/odom`, `/fix` |
| **输出话题** | `/fused_localization`(nav_msgs/Odometry), `/laser_localization`, `/local_map`, `/current_scan` |
| **更新频率** | 100Hz |
| **初始化方式** | 文件加载（上次位姿）/ GNSS / Scan Context 回环检测 |
| **地图格式** | 预分割子图 PCD 文件 + submap.csv 索引 + map_info.yaml（经纬度原点） |
| **关键依赖** | PCL, Eigen3, small_gicp, Sophus, GeographicLib |
| **与 task_manager 关系** | 提供 `/fused_localization`，是所有路径计算的位姿数据源 |

### 5.2 common_msgs

| 项目 | 详情 |
|------|------|
| **职责** | 定义 AGV 系统内所有自定义 ROS 消息类型 |
| **消息数量** | 9 个 msg，0 个 srv |
| **核心消息** | `nav_data`（32+ 字段）, `rcs_cmd_data`, `ipc_res_data`（60+ 字段） |
| **PLC 相关** | `plc_res_nav_data`, `plc_res_action_data`, `action_data`, `one_action_task` |
| **辅助消息** | `mapping_order`, `trans` |
| **头文件** | `common_haeder.h`（枚举 + 结构体 + ErrorManager 单例）, `common_time.h` |
| **被依赖** | task_manager, rcs_interaction, plc_interaction |

### 5.3 costmap_2d

| 项目 | 详情 |
|------|------|
| **职责** | 构建和维护 2D 栅格代价地图，用于碰撞检测和路径规划 |
| **层级插件** | StaticLayer + ObstacleLayer + VoxelLayer + InflationLayer + **FreeLayerBounds（自定义新增）** |
| **自定义扩展** | `FreeLayerBounds`：动态扩展代价地图边界，跟随机器人位置，默认 200m × 200m |
| **代价值** | FREE_SPACE=0, INSCRIBED_INFLATED_OBSTACLE=253, LETHAL_OBSTACLE=254, NO_INFORMATION=255 |
| **膨胀算法** | BFS 距离传播 + 指数衰减: `cost = 252 × exp(-weight × (d - inscribed_radius))` |
| **与 task_manager 关系** | 提供 `getCost()` / `transformFootprint()` / `convexFillCells()` 用于障碍检测；提供地图给 `planner_->makePlan()` |

### 5.4 global_planner

| 项目 | 详情 |
|------|------|
| **职责** | 全局路径规划（大量自定义扩展，远超标准 ROS 版本） |
| **规划算法** | Dijkstra（默认）/ A* / **Frenet 规划器（自定义新增）** |
| **路径提取** | GradientPath（梯度插值）/ GridPath（栅格贪心） |
| **自定义扩展** | ① Frenet 规划器（五次多项式轨迹）② 安全走廊生成 ③ B-spline 轨迹优化（NLopt）④ 曲率优化 ⑤ 额外方向模式(Leftward/Rightward) |
| **发布话题** | `~plan`, `~smooth_plan`, `~potential`, `~safety_corridors` |
| **插件注册** | `nav_core::BaseGlobalPlanner` → `bgp_plugin.xml` |
| **与 task_manager 关系** | 通过 `planner_->makePlan()` 被调用，在避障时生成绕障路径 |

### 5.5 nav_core

| 项目 | 详情 |
|------|------|
| **职责** | 定义导航栈的三个核心抽象接口（纯头文件库） |
| **接口** | `BaseGlobalPlanner`（makePlan）, `BaseLocalPlanner`（ComputeVelocityCommands）, `RecoveryBehavior`（runBehavior） |
| **注意** | 接口签名已修改（`BaseLocalPlanner` 增加了 `twist_current` 和 `pose_current` 参数），与标准 ROS 不兼容 |

### 5.6 navfn

| 项目 | 详情 |
|------|------|
| **职责** | Dijkstra/A* 全局规划器，`nav_core::BaseGlobalPlanner` 的具体实现 |
| **核心算法** | Dijkstra 波前传播（三级优先队列）/ A*（欧氏启发式） |
| **路径提取** | 梯度下降 + 双线性插值（亚像素精度）+ 振荡检测 |
| **服务** | `makeNavPlan`（srv） |
| **与 task_manager 关系** | 作为默认全局规划器被 `pluginlib` 动态加载 |

### 5.7 mpc_path_follower

| 项目 | 详情 |
|------|------|
| **职责** | 基于 MPC 的局部路径跟踪器（实验性，**当前代码中已注释未启用**） |
| **车辆模型** | 非线性自行车模型（前轮转向） |
| **优化器** | IPOPT + CppAD 自动微分 |
| **预测窗口** | N=20 步, dt=0.1s（2 秒） |
| **状态变量** | x, y, ψ, v, cte（横向偏差）, eψ（航向偏差） |
| **限速** | 硬编码 0.2 m/s 上限 |
| **已知问题** | 仅适合小曲率路径；eψ 符号可能有问题；代码注释 task_manager.cpp:99-110 |

### 5.8 plc_interaction

| 项目 | 详情 |
|------|------|
| **职责** | ROS ↔ PLC 双向 UDP 通信桥梁 |
| **协议** | 自定义二进制 UDP（非 Modbus） |
| **连接** | 导航端口 1101 + 执行器端口 1103，PLC IP 默认 192.168.2.69 |
| **线程架构** | 4 线程（主线程 100Hz + 导航 UDP 收 + 执行器 UDP 收 + Socket 管理） |
| **发送到 PLC** | 导航指令（~420 字节：速度/偏差/前瞻点/避障标志）+ 执行器指令（~64 字节：动作类型/值/ID） |
| **从 PLC 接收** | 速度/加速度/IMU/编码器/电池/GNSS/安全激光/楼层/停车状态/充电状态 |
| **发布话题** | `encoder`, `vel`, `imu_plc`, `odom`, `plc_res_nav_data`, `plc_res_action_data` |
| **订阅话题** | `nav_data`（来自 task_manager）, `action_data`（来自 task_manager） |

### 5.9 rcs_interaction

| 项目 | 详情 |
|------|------|
| **职责** | AGV ↔ RCS 调度系统 Modbus TCP 通信桥梁 |
| **协议** | Modbus TCP（标准工业协议） |
| **角色** | Modbus Server（端口 1500，等待 RCS 连接） |
| **寄存器映射** | 保持寄存器 2500 个（RCS→IPC 命令）+ 输入寄存器 500 个（IPC→RCS 状态） |
| **线圈** | 实时通信标志、急停、任务就绪、系统就绪 |
| **任务指令流** | RCS 写保持寄存器 → rcs_interaction 读取解析 → 发布 `task_rcs` 话题 → task_manager |
| **状态反馈流** | task_manager → `ipc_res_data` 话题 → rcs_interaction 订阅 → 写入 Modbus 输入寄存器 → RCS 读取 |

### 5.10 hipnuc_imu

| 项目 | 详情 |
|------|------|
| **职责** | HiPNUC IMU 串口驱动 |
| **通信** | 串口（/dev/ttyUSB0, 115200bps） |
| **数据包** | 0x91（浮点 IMU）+ 0x92（整型 IMU）+ 0x81（INS + GNSS） |
| **发布话题** | `/IMU_data`（sensor_msgs/Imu） |
| **与系统关系** | 为 `3d_localization_csg` 提供 IMU 原始数据 |

### 5.11 rslidar_sdk

| 项目 | 详情 |
|------|------|
| **职责** | RoboSense 官方 LiDAR 驱动 SDK（未修改） |
| **当前使用型号** | RSHELIOS + RSHELIOS_16P |
| **点云格式** | XYZIRT（x, y, z, intensity, ring, timestamp） |
| **发布话题** | `/rslidar_points`（PointCloud2） |
| **与系统关系** | 为 `3d_localization_csg` 和 `costmap_2d` 提供点云数据 |

### 5.12 vanjee_lidar_sdk

| 项目 | 详情 |
|------|------|
| **职责** | 万集官方 LiDAR 驱动 SDK（未修改） |
| **支持型号** | 14 种（719 系列、720 系列、722 系列、750、760 等） |
| **点云格式** | XYZIRT |
| **发布话题** | `/vanjee_points*`（PointCloud2），可选 IMU 和 LaserScan |
| **与系统关系** | 为 `3d_localization_csg` 和 `costmap_2d` 提供点云数据 |

---

## 6. 关键跨包调用汇总表

| task_manager 函数 | 源码行号 | 调用的外部包 | 具体调用方式 | 实现功能 |
|---|---|---|---|---|
| **构造函数** | :112 | `costmap_2d` | `new Costmap2DROS("global_costmap")` | 创建全局代价地图 |
| **构造函数** | :116 | `nav_core` + `navfn`/`global_planner` | `bgp_loader_.createInstance()` + `planner_->initialize()` | 动态加载全局规划器插件 |
| **构造函数** | :73 | `3d_localization_csg` | 订阅 `/fused_localization` | 获取 100Hz 融合定位 |
| **构造函数** | :76 | `rcs_interaction` | 订阅 `task_rcs` | 接收 RCS 任务指令 |
| **构造函数** | :78-79 | `plc_interaction` | 订阅 `plc_res_nav_data` + `plc_res_action_data` | 接收 PLC 底盘/执行器反馈 |
| **GenerateGlobalPath()** | :162 | `navfn`/`global_planner` | `planner_->makePlan(start, goal, plan)` | 在代价地图上规划绕障路径 |
| **obstacleDetection()** | :2666-2713 | `costmap_2d` | `getCost()`, `transformFootprint()`, `convexFillCells()` | 沿路径检测足迹覆盖区域的障碍物 |
| **findNearestObstacle()** | :2603 | `costmap_2d` | `getCostmap()`, `worldToMap()`, `getCost()`, `mapToWorld()` | 搜索机器人周围最近的障碍点 |
| **GenerateAndPubFollowPathData()** | :1500 | `plc_interaction` | `pub_nav_data_.publish()` | 发送路径跟踪导航指令到 PLC |
| **PubActionData()** | :1230 | `plc_interaction` | `pub_action_data_.publish()` | 发送执行器指令到 PLC |
| **PubIpcResData()** | :531 | `rcs_interaction` | `pub_ipc_res_data_.publish()` | 上报 AGV 完整状态到 RCS |
| **GenerateAndPubPickCargoData()** | :1787 | RFID 硬件 | `rfid_.startScan()` | 启动 RFID 异步扫描 |
| **GenerateAndPubPickCargoData()** | :1816 | M4 相机 | `p_pallet_identify_->GetPalletIdentifyData()` | 获取托盘精确位姿 |
| **FinishedCurrentStepTask()** | :342 | RFID 硬件 | `rfid_.stopScan()` + `rfid_.getResult()` | 停止扫描并比对 EPC 标签 |
| **Current3dPoseProcess()** | :2501 | `3d_localization_csg` | 回调处理 `/fused_localization` | 更新全局位姿和速度 |
| **TaskProcess()** | :2061 | `rcs_interaction` + `common_msgs` | 回调处理 `task_rcs` | 解析二进制任务数据 |
| **ProcessPlcResData()** | :2121 | `plc_interaction` | 回调处理 `plc_res_nav_data` | 更新底盘状态（电池/速度/到站/充电/楼层） |
| **ProcessPlcResActionData()** | :2132 | `plc_interaction` | 回调处理 `plc_res_action_data` | 更新执行器状态（完成标志/故障码） |

---

## 7. 架构特点与重设计建议方向

### 当前架构的优势

1. **分层清晰**: 感知 → 定位 → 规划 → 决策 → 执行，各层通过 ROS Topic 解耦
2. **双通信协议**: RCS 用标准 Modbus TCP（工业兼容），PLC 用自定义 UDP（低延迟）
3. **插件化规划器**: 通过 `pluginlib` 可热切换不同规划算法
4. **多传感器冗余**: 双品牌 LiDAR 支持（RoboSense + 万集）
5. **完整的任务状态机**: 8 状态覆盖正常、暂停、终止等场景

### 当前架构的不足

1. **task_manager 职责过重**: 2744 行单文件，同时承担任务解析、路径跟踪、避障决策、偏差计算、状态上报、RFID 管理、相机交互等职责，违反单一职责原则
2. **nav_core 接口已修改**: `BaseLocalPlanner` 签名与标准 ROS 不兼容，限制了第三方规划器的接入
3. **MPC 局部规划器未启用**: 代码已注释，路径跟踪完全依赖 PLC 侧的控制逻辑
4. **避障逻辑嵌入路径跟踪函数**: 避障状态机使用 `static` 变量维护状态，难以测试和维护
5. **硬编码较多**: 到站阈值、避障帧数阈值、7m 强制回归距离等散布在代码各处
6. **无行为树/任务编排框架**: 复杂任务逻辑用 switch-case 实现，扩展性受限
7. **缺少单元测试**: 核心逻辑无自动化测试覆盖

### 重设计建议方向

1. **拆分 task_manager**: 将路径跟踪、避障决策、状态上报、硬件交互拆分为独立模块/节点
2. **引入行为树**: 用 BehaviorTree.CPP 等框架替代 switch-case 任务编排
3. **恢复 nav_core 标准接口**: 使 MPC 等第三方局部规划器可以即插即用
4. **参数化配置**: 将所有阈值、策略参数集中到 YAML 配置文件
5. **考虑 ROS2 迁移**: 利用 lifecycle node、QoS、DDS 等特性提升系统可靠性
6. **增加监控层**: 独立的系统健康监控节点，统一管理故障检测和恢复

---

> 文档结束。本文档基于对 `test_ws/src/` 下所有功能包源码的逐行阅读生成，可作为架构重设计的参考基线。
