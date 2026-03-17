# AGV 新架构设计方案

> 基于现有系统逐行代码分析，针对性重设计
> 设计日期: 2026-03-16

---

## 1. 设计原则

| 原则 | 说明 |
|------|------|
| **单一职责** | 每个节点/模块只做一件事，现有 task_manager 的 2744 行拆为 6 个独立节点 |
| **接口标准化** | 恢复标准 nav_core / nav2 接口，第三方规划器可即插即用 |
| **配置集中化** | 所有阈值、策略参数统一 YAML 管理，消灭硬编码 |
| **可测试性** | 核心逻辑与 ROS 解耦，支持单元测试 |
| **渐进式迁移** | 保持 ROS1 兼容，架构设计同时考虑未来 ROS2 迁移路径 |

---

## 2. 新架构总览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              RCS 调度系统                                    │
└──────────────────────────────┬──────────────────────────────────────────────┘
                               │ Modbus TCP
                               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        通信适配层 (Communication Layer)                       │
│                                                                             │
│  ┌─────────────────────┐                    ┌─────────────────────────┐     │
│  │  rcs_bridge         │                    │  plc_bridge             │     │
│  │  (原 rcs_interaction│                    │  (原 plc_interaction    │     │
│  │   基本不变)          │                    │   基本不变)              │     │
│  └─────────┬───────────┘                    └────────────┬────────────┘     │
└────────────┼─────────────────────────────────────────────┼──────────────────┘
             │                                             │
             ▼                                             ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         决策编排层 (Decision Layer)                           │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    task_coordinator (新)                              │   │
│  │         职责: 纯任务状态机 + 子任务调度 + RCS 状态上报                   │   │
│  │         ~400 行，只管"做什么"，不管"怎么做"                              │   │
│  └──────┬────────────────┬──────────────────┬──────────────┬───────────┘   │
│         │                │                  │              │               │
│         ▼                ▼                  ▼              ▼               │
│  ┌─────────────┐ ┌─────────────┐  ┌──────────────┐ ┌──────────────┐      │
│  │ BT executor │ │ BT executor │  │ BT executor  │ │ BT executor  │      │
│  │ (导航行为树) │ │ (货叉行为树) │  │ (取货行为树)  │ │ (充电行为树)  │      │
│  └──────┬──────┘ └──────┬──────┘  └──────┬───────┘ └──────┬───────┘      │
└─────────┼───────────────┼────────────────┼────────────────┼───────────────┘
          │               │                │                │
          ▼               ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        执行能力层 (Capability Layer)                          │
│                                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │ path_tracker │  │ obstacle_mgr │  │ actuator_mgr │  │ sensor_mgr   │   │
│  │  (路径跟踪)   │  │  (避障管理)   │  │  (执行器管理) │  │ (传感器管理)  │   │
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        规划层 (Planning Layer)                                │
│                                                                             │
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐        │
│  │ global_planner   │  │ local_planner    │  │ path_generator     │        │
│  │ (pluginlib 加载)  │  │ (MPC / PurePur.) │  │ (稠密路径插值)      │        │
│  └──────────────────┘  └──────────────────┘  └────────────────────┘        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        感知定位层 (Perception Layer)  [基本不变]               │
│                                                                             │
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐        │
│  │ 3d_localization  │  │ costmap_2d       │  │ LiDAR / IMU 驱动   │        │
│  │ (ESKF + GICP)    │  │ (分层代价地图)    │  │ (rslidar/vanjee/   │        │
│  │                  │  │                  │  │  hipnuc)           │        │
│  └──────────────────┘  └──────────────────┘  └────────────────────┘        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 核心节点拆分详解

### 现有 task_manager (2744行) → 拆为 6 个节点

```
原 task_manager 职责分解:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

职责                          原代码行号            → 新归属节点
──────────────────────────────────────────────────────────────
任务状态机 (RunCycle)         1948-2033             → task_coordinator
任务接收解析 (TaskProcess)    2061-2119             → task_coordinator
二进制数据解码 (ReadNode...)  2167-2475             → task_coordinator
子任务完成判定 (Finished...)  257-409               → task_coordinator
子任务推进 (Updata...)        411-438               → task_coordinator
RCS 状态上报 (PubIpcRes...)   440-532               → task_coordinator
路径校验 (CheckRcsPath...)    535-554               → task_coordinator
──────────────────────────────────────────────────────────────
路径跟踪+偏差计算             1316-1510, 959-1206   → path_tracker
前瞻点计算 (GetLookahead...)  746-772               → path_tracker
稠密路径生成 (GenerateGlob..) 630-698               → path_tracker
──────────────────────────────────────────────────────────────
避障状态机 (4阶段)            1316-1510 (内嵌)      → obstacle_manager
障碍检测 (obstacleDetection)  2663-2731             → obstacle_manager
最近障碍搜索 (findNearest..)  2599-2662             → obstacle_manager
绕障路径规划 (GenerateGlob..) 146-167               → obstacle_manager
──────────────────────────────────────────────────────────────
货叉控制 (GenerateAndPubWork) 1754-1781             → actuator_manager
充电控制 (GenerateAndPubCh..) 1271-1301             → actuator_manager
原地自旋 (GenerateAndPubAr..) 1678-1751             → actuator_manager
──────────────────────────────────────────────────────────────
RFID 管理 (startScan/stop..)  构造函数+1784-1836    → sensor_manager
M4 相机管理 (PalletIdentify)  构造函数+1784-1836    → sensor_manager
──────────────────────────────────────────────────────────────
任务执行编排 (ExecuteTask)    1864-1932             → BehaviorTree
```

---

## 4. 各新节点详细设计

### 4.1 task_coordinator — 任务协调器（原 task_manager 核心状态机）

```
职责: 只管"做什么"，不管"怎么做"
预估代码量: ~400 行（vs 原 2744 行）

输入:
  ├── /task_rcs (rcs_bridge)          → 接收 RCS 任务
  ├── /task_feedback (各执行节点)      → 接收子任务完成反馈
  └── /plc_feedback (plc_bridge)      → 接收 PLC 状态

输出:
  ├── /ipc_res_data (→ rcs_bridge)    → 上报 AGV 状态
  ├── /task_command (→ BT executor)   → 下发子任务执行命令
  └── /task_visualization (→ RViz)    → 任务可视化
```

**状态机保持不变（已验证可靠）**:
```
NONE → NO_TASK → RECVING → EXECUTING → COMPLETED
                                     → PAUSED → ABORTED
```

**关键改变**:
- 不再直接调用任何硬件/传感器
- 不再计算路径偏差
- 不再管理避障状态
- 只负责: 解析任务 → 下发子任务 → 监听反馈 → 推进状态 → 上报 RCS

### 4.2 path_tracker — 路径跟踪节点

```
职责: 纯路径跟踪 + 偏差计算 + PLC 导航数据发送
预估代码量: ~500 行

输入:
  ├── /tracking_path (obstacle_manager)   → 当前应跟踪的路径（原始或绕障）
  ├── /fused_localization (3d_loc)        → 实时位姿
  └── /tracking_params (task_coordinator) → 跟踪参数（速度限制、到站阈值等）

输出:
  ├── /nav_data (→ plc_bridge)            → 导航控制指令
  ├── /path_deviation (→ obstacle_mgr)    → 实时偏差数据
  └── /tracking_status (→ task_coord.)    → 到站/脱轨/正常

服务:
  └── /generate_dense_path (srv)          → 稠密路径生成
```

**核心类**:
```cpp
class PathTracker {
public:
    // 从 YAML 加载所有参数（消灭硬编码）
    struct Config {
        double lookahead_min;          // 原 distance_lookahead_ 最小值
        double lookahead_max;          // 原 distance_lookahead_ 最大值
        double lookahead_speed_gain;   // 速度-前瞻距离增益
        double derail_threshold;       // 原 dis_derail_ (0.2m)
        double dense_path_step;        // 原 0.1m 步长
        double finish_distance;        // 原到站阈值
        double finish_velocity;        // 原 0.001 m/s
    };

    // 对外接口，清晰的输入-输出
    TrackingResult track(const Path& path,
                         const Pose& current_pose,
                         const Twist& current_twist);

private:
    PathDeviation calculateLineDeviation(/*...*/);
    PathDeviation calculateCurveDeviation(/*...*/);
    Point getLookaheadPoint(/*...*/);
};
```

### 4.3 obstacle_manager — 避障管理节点

```
职责: 独立的避障决策 + 障碍检测 + 路径切换
预估代码量: ~400 行

输入:
  ├── /original_path (task_coordinator)    → 原始任务路径
  ├── /fused_localization (3d_loc)         → 实时位姿
  └── /costmap (costmap_2d)                → 代价地图

输出:
  ├── /tracking_path (→ path_tracker)      → 当前应跟踪的路径
  ├── /obstacle_status (→ task_coord.)     → 障碍状态
  └── /avoidance_visualization (→ RViz)    → 避障可视化

依赖:
  └── global_planner (pluginlib)           → 绕障路径规划
```

**避障状态机（从 static 变量提升为正式类）**:
```cpp
// 原来: GenerateAndPubFollowPathData() 中 300 行 + static 变量
// 现在: 独立类，可单元测试

class ObstacleStateMachine {
public:
    enum State {
        FOLLOW_ORIGINAL,
        AVOIDANCE_PLANNING,
        FOLLOW_AVOIDANCE,
        RETURN_TO_ORIGINAL
    };

    struct Config {
        int detect_confirm_frames;       // 原硬编码 5 帧
        int replan_confirm_frames;       // 原硬编码 3 帧
        int overcome_confirm_frames;     // 原硬编码 2 帧
        double force_return_distance;    // 原硬编码 7m
    };

    // 每周期调用，返回应跟踪的路径
    Path update(const Path& original,
                const Pose& current,
                const Costmap2D& costmap);

    State getState() const;

private:
    State state_ = FOLLOW_ORIGINAL;
    int obstacle_counter_ = 0;
    // ... 干净的状态管理，无 static 变量
};
```

### 4.4 actuator_manager — 执行器管理节点

```
职责: 统一管理所有执行器（货叉、充电、自旋）
预估代码量: ~300 行

输入:
  ├── /actuator_command (BT / task_coord.)  → 执行器指令
  ├── /plc_res_action_data (plc_bridge)     → PLC 执行器反馈
  └── /fused_localization (3d_loc)          → 位姿（自旋用）

输出:
  ├── /action_data (→ plc_bridge)           → PLC 执行器控制
  ├── /nav_data (→ plc_bridge)              → 自旋时的导航数据
  └── /actuator_status (→ task_coord.)      → 完成/故障反馈

服务:
  ├── /fork_control (srv)     → 货叉升降
  ├── /charge_control (srv)   → 充电控制
  └── /spin_control (srv)     → 原地旋转
```

**自旋控制改进**:
```cpp
class SpinController {
public:
    struct Config {
        double angle_tolerance;   // 到位角度容差
        double max_angular_vel;   // 最大角速度
    };

    // 返回剩余角度，由 PLC 决定转速
    SpinResult update(double current_yaw);

private:
    double start_yaw_;
    double target_angle_;
    double accumulated_angle_ = 0.0;
    double last_yaw_;
    // unwrap 逻辑封装在内部
};
```

### 4.5 sensor_manager — 传感器管理节点

```
职责: 统一管理非导航传感器（RFID、M4 相机）
预估代码量: ~200 行

服务:
  ├── /rfid_scan (srv)            → 启动/停止 RFID 扫描，返回 EPC 列表
  ├── /pallet_identify (srv)      → 请求托盘位姿识别
  └── /sensor_status (srv)        → 查询传感器健康状态

发布:
  └── /pallet_pose (→ path_tracker)  → 实时托盘位姿（取货场景）
```

**改进点**:
- RFID 和相机初始化失败不影响其他功能（现有代码中是构造函数一起初始化）
- 传感器状态可独立监控
- 通过 Service 调用，调用方无需关心底层协议

### 4.6 system_monitor — 系统监控节点（新增）

```
职责: 统一健康监控 + 故障管理 + 诊断
预估代码量: ~250 行

监控项:
  ├── 各节点心跳检测 (bond)
  ├── 定位质量 (位姿跳变检测)
  ├── 通信链路 (RCS/PLC 连接状态)
  ├── 传感器状态 (LiDAR/IMU/RFID/Camera)
  └── 系统资源 (CPU/内存/磁盘)

输出:
  ├── /diagnostics (diagnostic_msgs)  → 标准 ROS 诊断
  ├── /system_error (→ task_coord.)   → 故障码（替代现有分散的 ErrorManager）
  └── /system_status (→ rcs_bridge)   → 系统就绪状态
```

---

## 5. 行为树设计（替代 ExecuteTask 的 switch-case）

### 5.1 为什么用行为树

| 现有方案 | 新方案 |
|---------|--------|
| `ExecuteTask()` 中 switch(type) 分发 | BehaviorTree.CPP 动态加载 XML 行为树 |
| 每种任务类型一个 `GenerateAndPub*()` 函数 | 每种行为一个 BT Action 节点 |
| 新任务类型需改 task_manager 源码 | 新任务类型只需新增 XML + Action 插件 |
| 无法可视化执行过程 | Groot2 实时可视化 + 回放 |

### 5.2 导航任务行为树示例

```xml
<!-- navigate_task.xml -->
<root BTCPP_format="4">
  <BehaviorTree ID="NavigateToGoal">
    <Sequence>
      <!-- 生成稠密路径 -->
      <GenerateDensePath path="{original_path}"
                         dense_path="{dense_path}"/>

      <!-- 主循环: 跟踪 + 避障 -->
      <ReactiveSequence>
        <!-- 并行监控障碍 -->
        <ObstacleMonitor original_path="{dense_path}"
                         tracking_path="{current_path}"
                         obstacle_detected="{has_obstacle}"/>

        <!-- 路径跟踪（跟踪 current_path，可能是原始或绕障） -->
        <PathFollow path="{current_path}"
                    tolerance="{finish_tolerance}"/>
      </ReactiveSequence>

      <!-- 等待 PLC 停车确认 -->
      <WaitForPLCStop timeout_ms="5000"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 5.3 取货任务行为树示例

```xml
<!-- pick_cargo_task.xml -->
<root BTCPP_format="4">
  <BehaviorTree ID="PickCargo">
    <Sequence>
      <!-- 阶段1: 启动传感器 -->
      <Parallel success_count="2" failure_count="1">
        <StartRFIDScan/>
        <StartPalletIdentify/>
      </Parallel>

      <!-- 阶段2: 相机引导对位 -->
      <ReactiveSequence>
        <GetPalletPose pose="{pallet_pose}"/>
        <NavigateToPallet pose="{pallet_pose}"/>
      </ReactiveSequence>

      <!-- 阶段3: 等待PLC停车 -->
      <WaitForPLCStop/>

      <!-- 阶段4: RFID 验证 -->
      <Sequence>
        <StopRFIDScan/>
        <GetRFIDResult expected_epc="{expected_epc}"
                       result="{rfid_result}"/>
        <ReportRFIDResult result="{rfid_result}"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

### 5.4 BT Action 节点与现有代码的映射

| BT Action 节点 | 调用的能力层节点 | 对应原代码 |
|----------------|-----------------|-----------|
| `PathFollow` | path_tracker | `GenerateAndPubFollowPathData()` 的跟踪部分 |
| `ObstacleMonitor` | obstacle_manager | `GenerateAndPubFollowPathData()` 的避障部分 |
| `GenerateDensePath` | path_tracker (srv) | `GenerateGlobalDensePath()` |
| `ForkControl` | actuator_manager (srv) | `GenerateAndPubWorkData()` |
| `SpinInPlace` | actuator_manager (srv) | `GenerateAndPubAroundSpinData()` |
| `ChargeControl` | actuator_manager (srv) | `GenerateAndPubChargeData()` |
| `StartRFIDScan` | sensor_manager (srv) | `rfid_.startScan()` |
| `GetPalletPose` | sensor_manager | `p_pallet_identify_->GetPalletIdentifyData()` |
| `WaitForPLCStop` | plc_bridge (topic) | `g_status_stop_ == 2` 判断 |

---

## 6. 话题/服务通信矩阵

```
节点间通信设计（Topic 用 ──→，Service 用 ==→）

rcs_bridge ──/task_rcs──→ task_coordinator ──/ipc_res_data──→ rcs_bridge
                                │
                                │ /task_command (ActionLib Goal)
                                ▼
                          BT executor
                          ├──→ path_tracker       ──/nav_data──→ plc_bridge
                          ├──→ obstacle_manager   ──/tracking_path──→ path_tracker
                          ├==→ actuator_manager   ──/action_data──→ plc_bridge
                          └==→ sensor_manager

plc_bridge ──/plc_res_nav_data──→ task_coordinator
plc_bridge ──/plc_res_action_data──→ actuator_manager

3d_localization ──/fused_localization──→ path_tracker
3d_localization ──/fused_localization──→ obstacle_manager
costmap_2d (内部 API) ←── obstacle_manager
global_planner (pluginlib) ←── obstacle_manager

system_monitor ──/diagnostics──→ (所有节点通过 bond 监控)
system_monitor ──/system_error──→ task_coordinator
```

---

## 7. 目录结构

```
test_ws/src/
├── agv_bringup/                    ← 新增: 统一启动包
│   ├── launch/
│   │   ├── agv_full.launch         ← 全系统启动
│   │   ├── agv_navigation.launch   ← 仅导航子系统
│   │   └── agv_simulation.launch   ← 仿真模式
│   └── config/
│       ├── task_coordinator.yaml   ← 任务参数
│       ├── path_tracker.yaml       ← 路径跟踪参数（所有原硬编码阈值）
│       ├── obstacle_manager.yaml   ← 避障参数
│       ├── actuator_manager.yaml   ← 执行器参数
│       └── sensor_manager.yaml     ← 传感器配置
│
├── task_coordinator/               ← 新: 原 task_manager 的状态机部分
│   ├── include/
│   │   └── task_coordinator/
│   │       ├── task_coordinator.h
│   │       ├── task_parser.h       ← 原 ReadNodeFromRegister 逻辑
│   │       └── status_reporter.h   ← 原 PubIpcResData 逻辑
│   ├── src/
│   │   ├── task_coordinator_node.cpp
│   │   ├── task_coordinator.cpp    ← ~400 行
│   │   ├── task_parser.cpp         ← ~200 行
│   │   └── status_reporter.cpp     ← ~150 行
│   └── test/
│       ├── test_task_parser.cpp    ← 单元测试
│       └── test_state_machine.cpp
│
├── path_tracker/                   ← 新: 路径跟踪
│   ├── include/
│   │   └── path_tracker/
│   │       ├── path_tracker.h
│   │       ├── deviation_calculator.h   ← 直线/弧线偏差计算（纯数学，可测试）
│   │       ├── lookahead.h              ← 前瞻点计算
│   │       └── dense_path_generator.h   ← 稠密路径生成
│   ├── src/
│   │   ├── path_tracker_node.cpp
│   │   └── ...
│   └── test/
│       ├── test_deviation_calculator.cpp  ← 关键: 偏差计算单元测试
│       └── test_lookahead.cpp
│
├── obstacle_manager/               ← 新: 避障决策
│   ├── include/
│   │   └── obstacle_manager/
│   │       ├── obstacle_manager.h
│   │       ├── obstacle_detector.h      ← 原 obstacleDetection
│   │       └── avoidance_state_machine.h ← 4阶段状态机（独立类）
│   ├── src/
│   │   └── ...
│   └── test/
│       └── test_avoidance_fsm.cpp       ← 避障状态机单元测试
│
├── actuator_manager/               ← 新: 执行器管理
│   ├── include/
│   │   └── actuator_manager/
│   │       ├── actuator_manager.h
│   │       ├── fork_controller.h
│   │       ├── charge_controller.h
│   │       └── spin_controller.h
│   ├── src/
│   │   └── ...
│   └── test/
│       └── test_spin_controller.cpp
│
├── sensor_manager/                 ← 新: 非导航传感器管理
│   ├── include/
│   │   └── sensor_manager/
│   │       ├── rfid_service.h
│   │       └── pallet_identify_service.h
│   └── src/
│       └── ...
│
├── agv_behavior_trees/             ← 新: 行为树定义
│   ├── trees/
│   │   ├── navigate.xml
│   │   ├── spin.xml
│   │   ├── fork_work.xml
│   │   ├── pick_cargo.xml
│   │   └── charge.xml
│   ├── include/
│   │   └── agv_bt_nodes/          ← BT Action 节点实现
│   │       ├── path_follow_action.h
│   │       ├── obstacle_monitor_action.h
│   │       ├── fork_control_action.h
│   │       ├── rfid_scan_action.h
│   │       └── wait_plc_stop_action.h
│   └── src/
│       └── bt_executor_node.cpp
│
├── system_monitor/                 ← 新: 系统监控
│   └── ...
│
├── common_msgs/                    ← 保持，可能新增几个 srv 定义
├── rcs_interaction/                ← 基本不变，改名 rcs_bridge
├── plc_interaction/                ← 基本不变，改名 plc_bridge
├── 3d_localization_csg/            ← 不变
├── costmap_2d/                     ← 不变
├── global_planner/                 ← 不变
├── navfn/                          ← 不变
├── nav_core/                       ← 恢复标准接口（去掉自定义参数）
├── rslidar_sdk/                    ← 不变
├── vanjee_lidar_sdk/               ← 不变
└── hipnuc_imu/                     ← 不变
```

---

## 8. 参数配置示例

### path_tracker.yaml（消灭所有硬编码）

```yaml
path_tracker:
  # 前瞻点参数（原 GetLookaheadPoint 硬编码）
  lookahead:
    min_distance: 0.5          # 最小前瞻距离 (m)
    max_distance: 2.0          # 最大前瞻距离 (m)
    speed_gain: 1.5            # 速度增益系数

  # 到站判定（原 FinishedCurrentStepTask 硬编码）
  arrival:
    distance_threshold: 0.03   # 到站距离阈值 (m)
    transit_threshold: 0.5     # 连续通过距离阈值 (m)
    velocity_threshold: 0.001  # 停车速度阈值 (m/s)

  # 脱轨检测
  derail:
    lateral_threshold: 0.2     # 横向偏差阈值 (m)

  # 稠密路径
  dense_path:
    step_size: 0.1             # 插值步长 (m)
```

### obstacle_manager.yaml

```yaml
obstacle_manager:
  # 避障状态机参数（原 GenerateAndPubFollowPathData 硬编码）
  avoidance:
    detect_confirm_frames: 5       # 连续检测确认帧数
    replan_confirm_frames: 3       # 重规划确认帧数
    overcome_confirm_frames: 2     # 绕过确认帧数
    force_return_distance: 7.0     # 强制回归距离 (m)

  # 障碍检测
  detection:
    cost_threshold: 253            # INSCRIBED_INFLATED_OBSTACLE
    footprint_check_step: 1        # 沿路径每隔几个点检测一次
```

---

## 9. 迁移策略（渐进式，不停机）

### 阶段一: 提取纯计算模块（1-2 周）
```
目标: 不改变节点结构，只提取可测试的纯函数
步骤:
  1. 将 DeviationCalculator 提取为独立头文件库
  2. 将 SpinController 提取为独立头文件库
  3. 将 TaskParser (ReadNodeFromRegister) 提取为独立头文件库
  4. 为上述模块编写单元测试
效果: task_manager 代码量从 2744 → ~2000 行，核心算法已测试覆盖
风险: 零——纯重构，外部行为不变
```

### 阶段二: 拆分独立节点（2-3 周）
```
目标: 将 sensor_manager、actuator_manager 拆为独立节点
步骤:
  1. 创建 sensor_manager 节点，封装 RFID + M4 相机为 Service
  2. 创建 actuator_manager 节点，封装货叉/充电/自旋为 Service
  3. task_manager 改为通过 Service 调用（替代直接调用）
效果: task_manager 从 ~2000 → ~1200 行
风险: 低——Service 调用有超时保护
```

### 阶段三: 拆分路径跟踪和避障（2-3 周）
```
目标: 将 path_tracker、obstacle_manager 拆为独立节点
步骤:
  1. 创建 obstacle_manager 节点（带完整的避障状态机类）
  2. 创建 path_tracker 节点
  3. task_manager 不再直接发送 nav_data，改为向 path_tracker 下发跟踪命令
效果: task_manager 从 ~1200 → ~400 行，即 task_coordinator
风险: 中——需要仔细处理 100Hz 实时性
```

### 阶段四: 引入行为树（1-2 周）
```
目标: 用 BehaviorTree.CPP 替代 ExecuteTask() 的 switch-case
步骤:
  1. 集成 BehaviorTree.CPP 库
  2. 实现 BT Action 节点（包装各能力层的 Service/Topic 调用）
  3. 编写 XML 行为树
  4. task_coordinator 改为通过 BT executor 执行子任务
效果: 新增任务类型无需改代码，只需新增 XML
风险: 低——BT 框架已工业验证
```

---

## 10. 新旧架构对比

| 维度 | 现有架构 | 新架构 |
|------|---------|--------|
| **核心节点代码量** | 2744 行（task_manager.cpp） | ~400 行（task_coordinator.cpp） |
| **最大单函数** | GenerateAndPubFollowPathData: ~200行 | PathTracker::track(): ~50行 |
| **任务扩展方式** | 改 task_manager 源码 + 重编译 | 新增 XML 行为树 + Action 插件 |
| **避障状态管理** | static 局部变量 | 独立类 + 单元测试 |
| **参数管理** | 散布在代码各处的魔数 | 集中 YAML 配置 |
| **传感器耦合** | task_manager 直接管理 RFID/相机 | 独立 sensor_manager + Service |
| **故障监控** | ErrorManager 单例散布 | 独立 system_monitor 节点 |
| **可测试性** | 无单元测试 | 核心算法 100% 可测试 |
| **可视化** | 仅 RViz 路径显示 | RViz + Groot2 行为树实时监控 |
| **ROS2 迁移难度** | 高（巨型单节点） | 低（各节点独立迁移） |

---

> 以上为架构设计方案，保留了现有系统所有功能，同时解决了 task_manager 职责过重、硬编码、不可测试等核心问题。如需调整任何部分，随时告诉我。
