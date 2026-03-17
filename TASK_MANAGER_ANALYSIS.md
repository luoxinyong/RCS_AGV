# task_manager 功能包 — 深度架构分析

> 经3组独立代理全量代码阅读 + 源码行级验证

---

## 一、整体定位

task_manager 是整个 AMR 系统的**业务中枢**，实现：
- 接收 RCS 调度任务 → 解析为内部任务序列
- 驱动全局路径规划器 → 生成密集路径点
- 100Hz 实时计算路径偏差 → 发送导航数据给 PLC
- 协调执行机构（货叉/辊筒/托盘/牵引棒）
- 集成 RFID 校验和 M4 相机托盘识别
- 4 状态避障状态机 → 动态重规划

---

## 二、文件组成

| 文件 | 行数 | 职责 |
|------|------|------|
| `include/task_manager.h` | 844 | 全部枚举、结构体、TaskManager类定义 |
| `src/task_manager.cpp` | 2744 | 核心业务逻辑实现 |
| `src/task_manager_node.cpp` | 25 | ROS节点入口，100Hz主循环 |
| `src/m4_socket_client.cpp` | 468 | M4 dToF相机TCP客户端 |
| `src/pallet_identify_m4.cpp` | 93 | 托盘识别M4实现 |
| `src/rfid_reader.cpp` | — | RFID阅读器（RS232 + 后台盘存线程） |
| `include/rfid_reader.h` | 59 | RFIDReader类（双缓冲 + 静态回调桥接） |
| `include/pallet_identify_interface.h` | — | 托盘识别抽象基类 + PalletPose结构 |
| `include/pallet_identify_m4.h` | — | M4托盘识别实现类 |
| `config/config_task.yaml` | — | 运行参数配置 |
| `param/global_planner_params.yaml` | — | 全局规划器参数 |
| `param/global_costmap_params.yaml` | — | 全局代价地图参数 |
| `param/costmap_common_params.yaml` | — | 通用代价地图参数（传感器、轮廓、膨胀） |

---

## 三、枚举体系（task_manager.h）

### 3.1 IpcStatusEnum（行44-54）— IPC与RCS通信状态

| 值 | 名称 | 含义 | RCS可执行操作 |
|----|------|------|-------------|
| 0 | STATUS_NONE | 手动/初始化中 | 不可发送数据 |
| 1 | STATUS_NO_TASK | 无任务，等待新任务 | 可发新任务 |
| 2 | STATUS_RECVING_TASK | 正在接收任务 | 可发新任务 |
| 3 | STATUS_EXECUTING_TASK | 执行中 | 可暂停/终止 |
| 4 | STATUS_COMPLETED_TASK | 已完成 | 可发新任务 |
| 5 | STATUS_PAUSE_TASK_BY_IPC | IPC主动暂停（故障） | 可暂停/终止 |
| 6 | STATUS_PAUSE_TASK_BY_RCS | RCS暂停 | 可恢复/终止 |
| 7 | STATUS_ABORT_TASK | 已终止 | 可发新任务 |

### 3.2 ACTUATOR_CMD_ENUM（行57-80）— 17种执行机构命令

| 范围 | 类别 | 命令 |
|------|------|------|
| 0x11-0x18 | 辊筒 | 左取/左放、右取/右放、前取/前放、后取/后放 |
| 0x21-0x24 | 货叉 | 上升、下降、抱夹、松夹 |
| 0x31-0x32 | 顶升托盘 | 上升、下降 |
| 0x41-0x42 | 牵引棒 | 上升、下降 |

### 3.3 ACUATOR_STATUS_ENUM（行83-94）— 9种反馈状态

`IDLE(0x00)`, `LIFT_UP_DONE(0x01)`, `LIFT_DOWN_DONE(0x02)`, `LIFTING(0x04)`, `ROLLING(0x10)`, `ROLL_GET_DONE(0x11)`, `ROLL_PUT_DONE(0x12)`, `CLAMPING(0x20)`, `CLAMP_DONE(0x21)`

### 3.4 RCSNodeTypeEnum（行97-111）— 10种节点类型 + 哨兵

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | RCS_NODE_NONE | 空 |
| 1 | RCS_NODE_LINE | 直线行驶 |
| 2 | RCS_NODE_CURVE | 弯道行驶 / 原地自旋(map_id=1) |
| 3 | RCS_NODE_WORK | 举升/装卸 |
| 4 | RCS_NODE_CHARGE | 充电 |
| 5 | RCS_NODE_TRANSFER | 转移 |
| 6 | RCS_NODE_AUTORUN_LINE | 自动直线 |
| 7 | RCS_NODE_AUTORUN_CURVE | 自动弯道 |
| 8 | RCS_NODE_STANDBY_OPE | 待机操作 |
| 9 | RCS_NODE_AUTORUN_BY_CAMERA | 相机引导取货 |
| 99 | RCS_NODE_TASK_FINISH | 任务完成标记 |

### 3.5 CurrExecuteTaskTypeEnum（行114-123）— 7种执行类型

`IPC_IDLE(0)`, `IPC_FOLLOW_PATH(1)`, `IPC_AROUND_SPIN(2)`, `IPC_PICK_CARGO(3)`, `IPC_WORK(4)`, `IPC_CHARGE(5)`, `IPC_TASK_FINISH(6)`

### 3.6 AvoidState（task_manager.cpp行3）— 4种避障状态

`FOLLOW_ORIGINAL(0)`, `AVOIDANCE_PLANNING(1)`, `FOLLOW_AVOIDANCE(2)`, `RETURN_TO_ORIGINAL(3)`

---

## 四、结构体体系（task_manager.h 行126-511）

### 4.1 路径偏差结构

**PathDeviation**（行126-132）：5个double字段
- `delta_x` — 纵向偏差（沿路径方向剩余距离）
- `delta_y` — 横向偏差（左侧为正）
- `delta_theta` — 角偏差（逆时针为正）
- `front_projection` — 车头投影纵向偏差
- `rear_projection` — 车尾投影纵向偏差

**DISREMAIN**（行133-137）：轻量版剩余距离(x, y, length)

### 4.2 任务节点结构体族

**7个`#pragma pack(push,1)`专用结构体 + 1个通用容器**：

```
RCSNodeStru（通用容器，行337-511，30+字段超集）
    ↕ 通过 Reset() 系列重载方法双向转换
    ├─ RCSLineNodeStru     (行141-166)  type=1  直线：start/end坐标、length
    ├─ RCSCurveNodeStru    (行168-192)  type=2  弯道：额外center坐标
    ├─ RCSWorkNodeStru     (行194-223)  type=3  工作：static_work、is_loading_cargo
    ├─ RCSChargeNodeStru   (行225-252)  type=4  充电：charge_mode、charge_limit
    ├─ RCSTransferNodeStru (行254-281)  type=5  转移：operator_id、cmd
    ├─ RCSStandbyNodeStru  (行283-308)  type=8  待机：Ope_type、adj_type、tar_point
    └─ RCSFinishNodeStru   (行310-335)  type=99 完成：task_id_index
```

**各结构体字段差异关键点**：
- 直线 vs 弯道：弯道多了 `center_pt_x/y`（圆心坐标）
- 工作：独有 `static_work`, `done_wait_time`, `is_loading_cargo`
- 充电：独有 `charge_mode`, `charge_limit`
- 待机：独有 `Ope_type`, `adj_type`, `motion_mehtod`, `limit_x/y`, `tar_point_x/y/theta`
- 相机取货（AUTORUN_BY_CAMERA）：通过RCSNodeStru的 `expected_epc_high/low`（行509-510）携带预期RFID码

**RCSNodeStru不是union**——它是包含所有字段的超集struct，通过Reset()方法从专用结构体复制对应字段。

---

## 五、TaskManager 类架构（行624-843）

### 5.1 成员变量分组

#### ROS接口（行719-737）
- 11个Publisher：nav_data, action_data, ipc_res_data, vel, 楼层号, 6个可视化
- 6个Subscriber：odom/pose, rcs_task, plc导航反馈, plc动作反馈, 门禁

#### 规划器子系统（行776-791）
- `planner_costmap_ros_` — Costmap2DROS指针
- `planner_` — 全局规划器(BaseGlobalPlanner)
- `bgp_loader_` — pluginlib加载器
- `mpc_planner_` / `mpc_loader_` — MPC本地规划器（可选）

#### 外设子系统（行776-841）
- `p_pallet_identify_` — 托盘识别器(unique_ptr<PalletIdentifyBase>)
- `rfid_` — RFID阅读器实例
- `flag_rfid_enabled_`, `flag_rfid_scan_started_` — 扫描控制标志

#### 任务数据（行772-796）
- `vec_step_task_stru_` — 临时任务序列
- `vec_step_task_stru_global_` — 全局任务序列
- `vec_path_point_global_` — 每个子任务的密集路径点
- `index_current_step_task_` — 当前执行到第几个子任务
- `compelete_percent_` — 完成百分比

#### PLC反馈状态（行799-811）
- `g_vel_linear_` — 当前线速度
- `g_status_stop_` — 停车到位状态(0=未知,1=未到位,2=到位)
- `g_status_charge_` — 充电状态(0=未知,1=充电中,2=结束)
- `g_status_sensor_signal_` — 传感器到位信号
- `g_id_stop_node_` — 停车点节点ID
- `g_id_floor_` — 当前楼层
- `g_status/error/value/id_action_1` — 执行机构反馈

#### 自旋控制（行830-835）
- `spin_initialized_`, `spin_start_yaw_`, `spin_total_angle_` — 自旋角度累积

#### 配置参数（行739-764）
- `min_dis_ahead_point_`(0.6), `distance_lookahead_`(1.0), `scale_ahead_point_`(0.7)
- `step_generate_path_`(0.1), `threshold_arrive_des_`(1.0), `charge_arrive_des_`(0.03)
- `distance_head_`(3.0), `distance_tail_`(0.45), `dis_derail_`(0.2)

### 5.2 方法清单

#### 主循环与状态机
| 方法 | 行号 | 功能 |
|------|------|------|
| `RunCycle()` | 630 | 100Hz主循环，驱动IPC状态机 |
| `ExecuteTask()` | 681 | 根据当前子任务类型分发到对应Generate函数 |
| `UpdataCurrentStepTaskIndex()` | 695 | 检查并推进子任务索引 |
| `FinishedCurrentStepTask()` | 697 | 判定当前子任务是否完成 |
| `InitTaskData()` | 693 | 接收新任务时初始化所有执行变量 |

#### Generate系列 — 每个函数针对特定任务类型生成不同的PLC导航数据
| 方法 | 行号 | 参数 | 对应任务 |
|------|------|------|---------|
| `GenerateAndPubNoTaskData()` | 675 | 无 | 空闲/暂停/终止 |
| `GenerateAndPubFollowPathData(obs_area)` | 676 | 障碍区域 | 直线/弯道导航 |
| `GenerateAndPubChargeData(charge_mode, obs_area)` | 677 | 充电模式 | 充电对接 |
| `GenerateAndPubAroundSpinData(angle, obs_area, dir)` | 678 | 目标角度+方向 | 原地自旋 |
| `GenerateAndPubWorkData(static_work, obs_area)` | 679 | 静止/动态标志 | 举升/装卸 |
| `GenerateAndPubPickCargoData(obs_area)` | 680 | 障碍区域 | 相机引导取货 |

#### 路径偏差计算
| 方法 | 行号 | 参数差异 | 计算内容 |
|------|------|---------|---------|
| `CalculateLinePathDeviation(Point A,B, Pose, PathDev)` | 631-632 | 几何点+位姿 | 5维偏差(delta_x/y/θ + 头尾投影) |
| `CalculateLinePathDeviation(Point32 A,B,cur, DISREMAIN)` | 633-634 | Point32+当前点 | 轻量版剩余距离 |
| `CalculateCurvePathDeviation(center,start,end,pose,r,dev)` | 636-638 | 几何圆弧参数 | 径向/切向/角偏差 |
| `CalculateCurvePathDeviation(path_vec, pose, dev)` | 711-713 | 路径点序列 | 最近段偏差(避障路径用) |

#### 路径生成
| 方法 | 行号 | 功能 |
|------|------|------|
| `GenerateLinePoints()` | 662-663 | 直线等距采样 |
| `GenerateArcPoints()` | 664-665 | 圆弧参数化采样 |
| `GenerateGlobalDensePath()` | 666 | 遍历任务序列，按类型调用上两者 |
| `GenerateGlobalPath()` | 706 | 调用全局规划器(A*/Dijkstra)避障重规划 |

#### 避障检测
| 方法 | 行号 | 功能 |
|------|------|------|
| `obstacleDetection()` | 716-717 | 沿路径检查轮廓覆盖区域的costmap代价 |
| `isOvercomeObstacle()` | 714 | 判断是否已绕过障碍 |
| `findNearestObstacle()` | 715 | 查找最近障碍物边界点 |
| `detectPathDeviation()` | 709-710 | 检测避障路径与原始路径的偏离程度 |

#### 辅助方法
| 方法 | 行号 | 功能 |
|------|------|------|
| `GetLookaheadPoint()` | 682 | 沿路径查找前视点 |
| `GetHeadTailPose()` | 691-692 | 根据yaw计算车头车尾坐标 |
| `GetCurrAndNextTaskInfo()` | 685-688 | 提取当前和下一子任务的13个信息字段 |
| `GetDisData()` | 689 | 计算到停止点和类型切换点的距离 |
| `CalCurveSignRadius()` | 655-657 | 根据起终点和圆心计算带符号半径 |
| `ReadNodeFromRegister()` | 658 | 从uint16寄存器数组解析RCSNodeStru |
| `FillCommonNavData()` | 671 | 填充nav_data的公共字段 |
| `CheckNextTaskIsStopType()` | 683 | 判断下一子任务是否需要停车 |
| `PubActionData()` | 674 | 发布执行机构命令 |
| `PubIpcResData()` | 660 | 发布AGV状态反馈给RCS |
| `PubFloorNum()` | 659 | 发布当前楼层号 |
| `SetErrorCode()/ClearErrorCode()` | 672-673 | 错误码位操作 |

---

## 六、整体逻辑流程

### 6.1 RunCycle() 状态机（行1948-2033）

```
STATUS_NONE
    │ (自动转换)
    ▼
STATUS_NO_TASK ──────── 等待 cmd_rcs == TASK_CMD_RECV
    │                   ├─ 将临时任务赋值给全局任务
    │                   ├─ CheckRcsPathTaskValidity() 校验连续性
    │                   ├─ GenerateGlobalDensePath() 生成密集路径
    │                   └─ ShowRcsTaskPoints() 可视化
    ▼
STATUS_RECVING_TASK ─── 等待 cmd_rcs == TASK_CMD_EXECUTE
    │                   └─ InitTaskData() 初始化执行变量
    ▼
STATUS_EXECUTING_TASK ─ 每帧调用 ExecuteTask()
    │                   ├─ 监听 TASK_CMD_PAUSE → STATUS_PAUSE_TASK_BY_RCS
    │                   └─ 任务全部完成 → STATUS_COMPLETED_TASK
    ▼
STATUS_COMPLETED_TASK ─ 自动转回 STATUS_NO_TASK

STATUS_PAUSE_TASK_BY_RCS
    ├─ TASK_CMD_EXECUTE → 恢复 STATUS_EXECUTING_TASK
    └─ TASK_CMD_OBORT → STATUS_ABORT_TASK → STATUS_NO_TASK

STATUS_ABORT_TASK
    ├─ 停止RFID扫描，清空EPC数据
    ├─ 发布NoTaskData
    └─ 重置百分比和自旋标志 → STATUS_NO_TASK
```

### 6.2 ExecuteTask() 任务分发（行1864-1932）

**这是一个完整的按任务类型分发的执行器**，每种节点类型走不同的处理分支：

```
switch(cur_step_task.type):

  RCS_NODE_LINE (1):
      task_type = IPC_FOLLOW_PATH
      → GenerateAndPubFollowPathData(obs_area)

  RCS_NODE_CURVE (2):
      if map_id == 1:     // map_id=1 表示自旋
          task_type = IPC_AROUND_SPIN
          解析heading编码→目标角度：
            heading ∈ [1820,∞): 右旋, angle=(heading-2000)°
            heading ∈ [800,1800): 左旋, angle=(heading-1000)°
          → GenerateAndPubAroundSpinData(angle_rad, obs_area, direction)
      else:               // 普通弯道
          task_type = IPC_FOLLOW_PATH
          → GenerateAndPubFollowPathData(obs_area)

  RCS_NODE_WORK (3):
      task_type = IPC_WORK
      → GenerateAndPubWorkData(static_work, obs_area)

  RCS_NODE_AUTORUN_BY_CAMERA (9):
      task_type = IPC_PICK_CARGO
      → GenerateAndPubPickCargoData(obs_area)

  RCS_NODE_CHARGE (4):
      task_type = IPC_CHARGE
      → GenerateAndPubChargeData(charge_mode, obs_area)

  RCS_NODE_TASK_FINISH (99):
      task_type = IPC_TASK_FINISH
      → GenerateAndPubNoTaskData()
```

每次调用后统一执行：
```
UpdataCurrentStepTaskIndex()  // 检查是否完成当前子任务，推进索引
每500帧显示一次路径可视化
```

### 6.3 Generate系列函数的差异本质

**这些函数不可合并，因为每种任务类型的PLC导航需求完全不同：**

#### GenerateAndPubFollowPathData()（行1316-1510）— 最复杂

```
1. 检查obs_area第14位判断避障区域
2. 根据子任务类型计算偏差：
   ├─ 直线任务 → CalculateLinePathDeviation(几何版)
   └─ 弯道任务 → CalculateCurvePathDeviation(几何版)
3. 【4状态避障状态机】完整运行（见6.4）
4. 若使用避障路径 → CalculateCurvePathDeviation(路径序列版)
5. 脱轨检测：delta_y > dis_derail_ → 设置错误码
6. 进电梯标志：cargo_size==2 → flag_enter_elevator
7. 距离数据填充：GetDisData()
8. 偏差数据填充：delta_center/head/tail/angle
9. 投影点计算：base/head/tail三个投影点
10. 前视点获取：GetLookaheadPoint()
```

**独有特征**：避障状态机、脱轨检测、投影点计算、前视点

#### GenerateAndPubAroundSpinData()（行1678-1751）

```
1. 首次调用：记录spin_start_yaw_，计算spin_total_angle_
2. 后续调用：
   ├─ 计算已旋转角度 = NormalizeAngle(current_yaw - spin_start_yaw_)
   └─ 剩余角度 = spin_total_angle_ - 已旋转
3. 填充 angle_diff_spin_around = remaining_angle
4. cmd_vel = {0, 0, 0}（速度由PLC根据角度差控制）
```

**独有特征**：角度累积计算、方向控制、无路径偏差概念

#### GenerateAndPubWorkData()（行1754-1781）

```
1. 所有距离字段 = 0（不需要导航距离）
2. cmd_vel = {0, 0, 0}（停车执行动作）
3. → PubActionData(ACTUATOR_CMD_FORK_DOWN, static_work, id_action_current_)
```

**独有特征**：下发执行机构命令、完全不涉及路径

#### GenerateAndPubChargeData()（行1271-1301）

```
1. 距离字段 = 0
2. flag_charge = charge_mode（0=未知, 1=充电, 2=停止充电）
3. cmd_vel = {0, 0, 0}
```

**独有特征**：充电模式标志，PLC根据此标志控制充电桩对接

#### GenerateAndPubPickCargoData()（行1784-1836）

```
1. 触发RFID扫描（首次进入时）：
   if (flag_rfid_enabled_ && !flag_rfid_scan_started_)
       rfid_.startScan()
2. 托盘识别数据获取：
   if (flag_enable_pallet_identify_)
       p_pallet_identify_->GetPalletIdentifyData(pallet_pose)
       填充 x_detect, y_detect, theta_detect
3. flag_pick_task_valid = 1
```

**独有特征**：RFID扫描触发、托盘识别集成

#### GenerateAndPubNoTaskData()（行1233-1269）

```
所有字段归零，cmd_vel = {0,0,0}
→ PubActionData(ACTUATOR_CMD_STOP, 0, 0)
```

**独有特征**：全清零，停止一切

### 6.4 避障状态机（行1347-1443，在FollowPathData内部）

```
FOLLOW_ORIGINAL ─────────────────────────────────
    │ obstacleDetection() 连续检测到障碍≥5次
    │ 且 obs_area标志允许避障
    ▼
AVOIDANCE_PLANNING ──────────────────────────────
    │ GenerateGlobalPath(goal=子任务终点)
    │ ├─ 规划成功 → 路径格式转换
    │ └─ 规划失败 → 回退原始路径
    ▼
FOLLOW_AVOIDANCE ────────────────────────────────
    │ 持续检测：
    │ ├─ 避障路径再次被堵(≥3次) → 回到 AVOIDANCE_PLANNING
    │ ├─ isOvercomeObstacle()(≥2次) → RETURN_TO_ORIGINAL
    │ └─ 剩余距离<7m → 强制 FOLLOW_ORIGINAL
    ▼
RETURN_TO_ORIGINAL ──────────────────────────────
    │ FindClosestPoint() 找原始路径最近点
    │ GenerateLinePoints() 生成连接路径
    │ ├─ 连接路径无障碍 → FOLLOW_ORIGINAL（平滑切回）
    │ └─ 连接路径有障碍 → FOLLOW_AVOIDANCE（继续绕）
```

### 6.5 任务完成判定（FinishedCurrentStepTask）

**IPC_FOLLOW_PATH**：
```
计算剩余距离 < threshold_arrive_des_(0.08m)
  ├─ 下一任务是停止类型 → 额外等待 g_vel_linear_ < 0.001
  └─ 下一任务是导航类型 → 直接完成
特殊：连接路径阈值=0.5m，充电/动作点=0.03m
```

**IPC_AROUND_SPIN**：
```
g_id_stop_node_ == cur_node.id_a  AND  g_status_stop_ == 2
```

**IPC_PICK_CARGO**：
```
g_status_stop_ == 2 (停车到位)
  → 停止RFID扫描 → 获取结果 → 与预期EPC比较
  → rfid_result: 1=匹配, 2=不匹配, 3=未扫到
  → 保存actual_epc_high/low
```

**IPC_WORK**：
```
g_status_action_1 ∈ {LIFT_UP_DONE, LIFT_DOWN_DONE}
  AND g_id_finish_action_1 == id_action_current_
```

**IPC_CHARGE**：
```
g_status_charge_ == 2 (充电结束)
```

---

## 七、辅助子系统

### 7.1 RFID阅读器（rfid_reader.h/cpp）

**设计特点**：
- RS232串口通信（`/dev/ttyUSB0:115200`）
- 静态实例指针桥接C回调（`static RFIDReader* instance_`）
- **双缓冲**：`active_buffer_`（采集中）→ `result_buffer_`（采集完成后锁定）
- `std::atomic<bool>` 控制扫描/停止

**工作流**：`init()` → `startScan()`(创建后台线程) → 回调写入active_buffer_ → `stopScan()`(锁定到result) → `getResult()`

### 7.2 M4托盘识别（m4_socket_client + pallet_identify_m4）

**设计特点**：
- TCP客户端连接M4 dToF相机（默认192.168.3.82:5501）
- 后台接收线程 + mutex保护数据
- 20字节数据帧：帧头(2B) + 类型(2B) + 符号(3B) + X/Y/Theta各4B
- 单位：微米→毫米(*0.001)，角度*0.001
- 有效性检查：Y偏差>0.5m认为误识别

**接口**：`PalletIdentifyBase`(抽象基类) → `PalletIdentifyM4`(M4实现)，便于扩展其他相机

### 7.3 寄存器解析（ReadNodeFromRegister, 行2167-2475）

按节点类型将uint16数组映射为对应的RCSNodeStru字段。关键细节：
- int32字段由两个uint16拼接
- `heading == -2` 表示后退（target_vel取反）
- CURVE节点：start==end 则为自旋（设map_id=1）
- AUTORUN_BY_CAMERA：额外解析 expected_epc_high/low（索引14-15）
- obs_area 复用 cargo_size 字段

---

## 八、配置参数详解

### config_task.yaml

```yaml
# 定位模式
flag_use_2d_loc: false           # false=3D融合定位(/fused_localization)

# 功能开关
flag_enable_mpc: false           # MPC控制器
flag_enable_pallet_identify: false # M4托盘识别
flag_show_project_point: true    # 可视化投影点

# 路径跟踪核心参数
min_dis_ahead_point: 1.0         # 前瞄最小距离(m)
scale_ahead_point: 0.7           # 前瞄距离与速度的缩放系数
step_generate_path: 0.1          # 密集路径点间距(m)

# 到达判定
threshold_arrive_des: 0.08       # 导航到位阈值(m)
charge_arrive_des: 0.03          # 充电/精定位到位阈值(m)

# 车体几何
distance_head: 1.0               # 车头距中心(m)
distance_tail: 1.0               # 车尾距中心(m)
dis_derail: 0.2                  # 脱轨报警阈值(m)

# 通信
topic_task_rcs: task_rcs
base_global_planner: "global_planner/GlobalPlanner"
ip_address_dtof: 192.168.3.82   # M4相机IP
```

### 代价地图配置

```yaml
# 全局代价地图
global_costmap:
  width: 200.0, height: 200.0    # 200m×200m
  resolution: 0.05               # 5cm/格
  rolling_window: true           # 跟随机器人
  update_frequency: 10           # 10Hz更新
  plugins:
    - ObstacleLayer              # 障碍物层
    - InflationLayer             # 膨胀层

# 轮廓和膨胀
footprint: [[0,-0.5],[0,0.5],[1.62,0.5],[1.62,-0.5]]
inflation_radius: 0.5
cost_scaling_factor: 5.0
```

---

## 九、话题接口汇总

### 订阅

| 话题 | 类型 | 来源 | 频率 |
|------|------|------|------|
| `/fused_localization` | Odometry | 3d_localization | 10Hz |
| `task_rcs` | rcs_cmd_data | rcs_interaction | 事件 |
| `plc_res_nav_data` | plc_res_nav_data | plc_interaction | ~50Hz |
| `plc_res_action_data` | plc_res_action_data | plc_interaction | 事件 |
| `coil_status` | Int8 | rcs_interaction | 事件 |

### 发布

| 话题 | 类型 | 目标 | 频率 |
|------|------|------|------|
| `nav_data` | nav_data | plc_interaction→PLC | 100Hz |
| `action_data` | action_data | plc_interaction→PLC | 按需 |
| `ipc_res_data` | ipc_res_data | rcs_interaction→RCS | 100Hz |
| `num_floor` | Int32 | 3d_localization | 变化时 |
| 6个可视化话题 | Marker/Path | rviz | 按需 |

---

## 十、后续架构搭建建议

### 已有设计的合理之处（应保留）

1. **ExecuteTask()的任务分发机制**：每种节点类型有独立的处理分支，逻辑清晰，易于扩展新任务类型
2. **Generate系列函数的分离**：每个函数填充不同的PLC数据字段、触发不同的外设操作，合并反而增加复杂度
3. **4状态避障状态机**：状态转换条件（连续检测次数、距离阈值）设计稳健
4. **RCSNodeStru通用容器 + Reset()互转**：兼顾了Modbus寄存器的平坦结构和业务逻辑的类型安全
5. **PalletIdentifyBase抽象接口**：良好的扩展设计
6. **RFID双缓冲**：避免采集和读取的竞态

### 应改进的方向

| 方向 | 现状 | 建议 | 理由 |
|------|------|------|------|
| **文件拆分** | 2744行单文件 | 按职责拆分为多个.cpp（状态机、偏差计算、避障、寄存器解析、设备管理） | 降低单文件认知负担，便于并行开发 |
| **未处理的节点类型** | TRANSFER/STANDBY_OPE/AUTORUN_LINE/CURVE 在ExecuteTask()中 default:break | 根据实际业务需求补充实现，或显式日志提示"该类型暂不支持" | 静默丢弃可能导致RCS侧困惑 |
| **heading编码** | heading字段复用编码（-2=后退, 1820+=右旋, 800-1800=左旋） | 考虑使用独立字段或结构化编码替代magic number | 可读性差，新开发者易误解 |
| **obs_area/cargo_size复用** | obs_area复用cargo_size字段传递 | 保持但添加清晰注释 | 已约定的协议不宜轻改 |
| **规划器调用时机** | GenerateGlobalPath()在FollowPathData内同步调用 | 评估是否需要异步化（当前若规划器响应快则无需） | 仅当实际出现100Hz循环被阻塞时才需要 |
| **错误恢复** | 设置错误码后仅上报RCS，无自恢复 | 根据业务需求评估是否需要添加自动重试/回退逻辑 | 取决于RCS侧是否已有重试机制 |
| **密集路径缓存** | GenerateGlobalDensePath()在接收任务时一次性生成 | 当前设计已经合理（非每帧重算），保持不变 | 之前文档的建议有误——实际是一次性生成 |

### 重搭建议的具体拆分方案

```
task_manager/
├── include/
│   ├── task_manager.h              # TaskManager类声明（精简）
│   ├── task_types.h                # 所有枚举和结构体定义
│   ├── path_deviation_calculator.h # 路径偏差计算
│   ├── obstacle_avoidance.h        # 避障状态机
│   ├── register_parser.h           # 寄存器解析逻辑
│   ├── device_manager.h            # RFID + 托盘识别管理
│   └── (现有的rfid/pallet头文件)
├── src/
│   ├── task_manager.cpp            # 主逻辑（状态机 + ExecuteTask + Generate系列）
│   ├── path_deviation_calculator.cpp
│   ├── obstacle_avoidance.cpp
│   ├── register_parser.cpp
│   ├── device_manager.cpp
│   └── (现有辅助文件)
```

**拆分原则**：
- ExecuteTask()和Generate系列保留在task_manager.cpp中（它们是业务核心，拆出去反而增加调用复杂度）
- 将**纯计算逻辑**（偏差计算、寄存器解析）和**设备管理**抽出为独立模块
- 避障状态机可独立——它有自己的状态和转换规则，与任务类型无关
