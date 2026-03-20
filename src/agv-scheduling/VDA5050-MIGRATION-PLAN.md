# VDA5050 通信迁移方案

> 目标：将调度系统(agv-scheduling) ↔ AGV 之间的所有通信从 Modbus TCP 迁移到 VDA5050 v2.0 标准协议（基于 MQTT）

---

## 一、架构对比

### 现有架构（Modbus TCP）

```
前端 → WebSocket → scheduler ──Modbus TCP──→ rcs_interaction(ROS) ──task_rcs──→ task_manager → PLC
                       ↑                          ↓
                   Modbus读状态              Modbus返回状态
```

### 目标架构（VDA5050 / MQTT）

```
前端 → WebSocket → scheduler ──VDA5050 Order/InstantAction──→ vda5050_bridge(ROS) ──task_rcs──→ task_manager → PLC
                       ↑              (MQTT)                        ↓
                  VDA5050 State  ←──────────────────────────  VDA5050 State
```

**不变的部分**：
- 前端 ↔ scheduler（WebSocket）
- task_manager（完全不改，仍然接收 task_rcs topic）
- plc_interaction（完全不改，仍然 UDP 通信 PLC）
- 电动门控制（仍然 scheduler ↔ IoT MQTT，不经过 AGV）

**替换的部分**：
- `rcs_interaction`（Modbus slave）→ `vda5050_bridge`（MQTT VDA5050 客户端）
- `scheduler` 中的 Modbus 读写 → VDA5050 MQTT 发布/订阅

---

## 二、任务类型完整映射

### 2.1 运动任务（通过 Edge 传递）

| type | 名称 | VDA5050 映射 | 参数传递方式 |
|------|------|-------------|-------------|
| **1** | LINE 直线 | **Edge**（无 trajectory） | `startNode.nodePosition` → 起点坐标；`endNode.nodePosition` → 终点坐标；`Edge.maxSpeed` → 速度；heading/distance 从坐标计算得出 |
| **2** | CURVE 弯道 | **Edge**（有 **trajectory**） | 起终点同上；**圆心坐标通过 Edge.trajectory (NURBS) 标准字段传递**——圆弧 = 2阶有理NURBS，3个控制点精确表示 |
| **6** | AUTORUN_LINE | Edge + Edge Action `autorun` | 同 LINE + `actionParameters: [{key:"depth", value:...}]` |
| **7** | AUTORUN_CURVE | Edge(有trajectory) + Edge Action `autorun` | 同 CURVE + depth |
| **9** | CAMERA 视觉巡线 | Edge + Edge Action `cameraFollow` | 同 LINE + `actionParameters: [{key:"epc_high",...}, {key:"epc_low",...}]` |

#### 关键：圆弧的 NURBS 表示

VDA5050 v2.0 标准中 Edge 有官方的 `trajectory` 字段（类型为 NURBS 曲线），不是自定义扩展：

```typescript
Edge.trajectory: {
  degree: 2,                    // 2阶有理NURBS = 圆锥曲线（含圆弧）
  knotVector: [0, 0, 0, 1, 1, 1],
  controlPoints: [
    { x: startX, y: startY, weight: 1.0 },           // P0 起点
    { x: tangentX, y: tangentY, weight: cos(θ/2) },   // P1 切线交点（从起终点和圆心算出）
    { x: endX, y: endY, weight: 1.0 }                 // P2 终点
  ]
}
```

**转换函数**（双向）：
- 调度端：`centerToNurbs(start, end, center)` → 圆心坐标转 NURBS 控制点
- AGV端：`nurbsToCenter(controlPoints)` → NURBS 控制点反算圆心坐标

### 2.2 作业任务（通过 Node Action 传递）

| type | 名称 | VDA5050 映射 | actionParameters |
|------|------|-------------|------------------|
| **3** | WORK/举升 | Node Action `liftHeight` | height, hspeed, stop_before_done, done_wait_time, cargo_size, is_loading_cargo, heading, position1/2/3 |
| **4** | CHARGE 充电 | Node Action `charge`（标准） | charge_mode, charge_limit |
| **5** | TRANSFER 中转 | Node Action `transfer` | heading, is_where, operator_id, position, obj_speed, cmd, wait_time, cargo_size |
| **8** | STANDBY 待命 | Node Action `waitForTrigger` | Ope_type, Sen_type, WaitTime, CodeID, cargo_size, tar_point_x/y/theta, adj_type, motion_method, limit_x/y |
| **99** | FINISH 结束 | Order 末尾隐含 | task_id_index 放最后一个 Node 的 actionParameters |

### 2.3 控制指令（通过 InstantAction 传递）

| 现有命令 | Modbus 方式 | VDA5050 映射 |
|---------|------------|-------------|
| 暂停 (TaskType.pause=1) | 写寄存器 0x0001 | InstantAction `startPause` |
| 执行 (TaskType.execute=2) | 写寄存器 0x0002 | Order 下发即执行 |
| 停止 (TaskType.stop=3) | 写寄存器 0x0003 | InstantAction `cancelOrder` |
| 解除等待 (TaskType.active=5) | 写寄存器 0x0005 | InstantAction `stopPause` |
| 停车 (coil=true) | writeSingleCoil(0, true) | InstantAction `startPause` |
| 解除停车 (coil=false) | writeSingleCoil(0, false) | InstantAction `stopPause` |

### 2.4 状态回传（通过 State 消息传递）

| 现有字段 | Modbus input寄存器 | VDA5050 State 映射 |
|---------|-------------------|-------------------|
| px, py, pa | offset 88-96 | `agvPosition: { x, y, theta, mapId, positionInitialized }` |
| tr_step_index, tr_cplt_percent | offset 28-30 | `nodeStates[]`, `edgeStates[]` 精确追踪 |
| step_cplt_percent, step_type | offset 34-36 | `actionStates[].actionStatus` |
| energy_level, status_charge | offset 108-110 | `batteryState: { batteryCharge, charging }` |
| traction_speed | offset 102 | `velocity: { vx }` |
| lift_height | offset 98 | 自定义信息字段或 Action 状态 |
| err1-5, warnings | offset 126-136 | `errors[]: { errorType, errorLevel }` |
| cargo_loaded | offset 138 | `loads[]: { loadId, loadType }` |
| traction_is_parked, step_en_pause | offset 104, 40 | `driving`, `paused` |
| tr_cmd, tr_type, tr_step_size | offset 0-16 | `orderId`, `orderUpdateId` |

### 2.5 电动门控制

电动门是 scheduler ↔ IoT设备 的通信（MQTT `/iot/fhjlm/...`），不属于调度↔AGV 范畴。

**处理方式**：scheduler 从 VDA5050 State 中读取 AGV 步骤进度（替代现有的 Modbus 读取），仍然自动触发开关门逻辑。门的 MQTT 通信保持不变。

---

## 三、MQTT Topic 规范

```
uagv/v2/{manufacturer}/{serialNumber}/order            ← scheduler 发布
uagv/v2/{manufacturer}/{serialNumber}/instantActions    ← scheduler 发布
uagv/v2/{manufacturer}/{serialNumber}/state             → vda5050_bridge 发布（1Hz）
uagv/v2/{manufacturer}/{serialNumber}/connection        → vda5050_bridge 发布（retained）
uagv/v2/{manufacturer}/{serialNumber}/visualization     → vda5050_bridge 发布（10Hz，可选）
uagv/v2/{manufacturer}/{serialNumber}/factsheet         → vda5050_bridge 发布（retained，可选）

/iot/fhjlm/sub/{doorIp}                                 ← scheduler 发布（门控，保持不变）
/iot/fhjlm/pub/{doorIp}                                 → IoT设备 发布（门状态，保持不变）
```

---

## 四、实施步骤

### 第一步：补充 VDA5050 类型定义（server 端）

在 `server/src/types/vda5050.ts` 中补充：
- `Trajectory` 接口（degree, knotVector, controlPoints）
- `ControlPoint` 接口（x, y, weight）
- `Edge.trajectory?: Trajectory` 字段

### 第二步：实现 NURBS ↔ 圆心 转换工具函数

```typescript
// server/src/utils/nurbs.ts
function centerToNurbs(start, end, center): Trajectory { ... }
function nurbsToCenter(trajectory: Trajectory): { cx, cy } { ... }
```

### 第三步：改造 scheduler（server 端）

1. `schedulingHandler()` 改为构造 VDA5050 Order JSON → 发布到 MQTT
2. `stopHandler()` 改为发布 VDA5050 InstantAction `cancelOrder`
3. `getAgvStatus()` 改为订阅 VDA5050 State topic（替代 Modbus 轮询）
4. 门控逻辑保持不变，改为从 VDA5050 State 读取步骤进度

### 第四步：新建 vda5050_bridge ROS 节点（AGV 端）

替代 `rcs_interaction`：
1. 订阅 MQTT VDA5050 Order → 翻译为 `common_msgs::rcs_cmd_data` → 发布到 `task_rcs` topic
2. 订阅 MQTT VDA5050 InstantAction → 翻译为对应的 ROS 控制指令
3. 读取 task_manager 状态 → 构造 VDA5050 State → 发布到 MQTT
4. NURBS 控制点 → 反算圆心坐标 → 填入 RCSCurveNodeStru

### 第五步：测试验证

1. 单元测试：NURBS ↔ 圆心 转换精度
2. 集成测试：scheduler → MQTT → vda5050_bridge → task_manager 全链路
3. 全任务类型测试：LINE / CURVE / WORK / CHARGE / TRANSFER / STANDBY / AUTORUN / CAMERA
4. 电动门联动测试：AGV 步骤切换触发开关门
5. 异常测试：MQTT 断连重连、Order 取消、AGV 离线

---

## 五、改动范围总结

| 模块 | 改动程度 | 说明 |
|------|---------|------|
| `server/src/types/vda5050.ts` | 小改 | 补充 Trajectory 类型 |
| `server/src/utils/nurbs.ts` | 新增 | NURBS ↔ 圆心 转换 |
| `server/src/services/scheduler.ts` | 重构 | Modbus 读写 → VDA5050 MQTT 发布/订阅 |
| `server/src/services/vda5050-adapter.ts` | 重构/合并 | 与 scheduler 合并为统一的 VDA5050 通信层 |
| `server/src/services/modbus.ts` | 保留 | 可保留用于调试，生产环境不再使用 |
| `vda5050_bridge` (ROS C++/Python) | 新增 | 替代 rcs_interaction |
| `rcs_interaction` | 弃用 | 被 vda5050_bridge 替代 |
| **task_manager** | **不改** | 仍接收 task_rcs topic |
| **plc_interaction** | **不改** | 仍 UDP 通信 PLC |
| **前端** | **不改** | 仍通过 WebSocket 和 scheduler 交互 |
| **电动门** | **不改** | scheduler 侧逻辑保持，数据源从 Modbus → VDA5050 State |

---

## 六、VDA5050 Order 示例

以"到五号点去"任务为例（5步：LINE→SPIN→LINE→SPIN→ARC）：

```json
{
  "headerId": 1,
  "timestamp": "2026-03-20T10:00:00.000Z",
  "version": "2.0.0",
  "manufacturer": "csg",
  "serialNumber": "1",
  "orderId": "order-20260320-001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "N1", "sequenceId": 0, "released": true,
      "nodePosition": { "x": 1.000, "y": 2.000, "theta": 0, "mapId": "1" },
      "actions": []
    },
    {
      "nodeId": "N2", "sequenceId": 2, "released": true,
      "nodePosition": { "x": 3.500, "y": 2.000, "theta": 1.5708, "mapId": "1" },
      "actions": []
    },
    {
      "nodeId": "N3", "sequenceId": 4, "released": true,
      "nodePosition": { "x": 3.500, "y": 5.000, "theta": 0.3142, "mapId": "1" },
      "actions": []
    },
    {
      "nodeId": "N5", "sequenceId": 6, "released": true,
      "nodePosition": { "x": 5.000, "y": 6.500, "theta": 0, "mapId": "1" },
      "actions": []
    }
  ],
  "edges": [
    {
      "edgeId": "E1-2", "sequenceId": 1, "released": true,
      "startNodeId": "N1", "endNodeId": "N2",
      "maxSpeed": 0.5,
      "actions": []
    },
    {
      "edgeId": "E2-3", "sequenceId": 3, "released": true,
      "startNodeId": "N2", "endNodeId": "N3",
      "maxSpeed": 0.5,
      "actions": []
    },
    {
      "edgeId": "E3-5", "sequenceId": 5, "released": true,
      "startNodeId": "N3", "endNodeId": "N5",
      "maxSpeed": 0.3,
      "trajectory": {
        "degree": 2,
        "knotVector": [0, 0, 0, 1, 1, 1],
        "controlPoints": [
          { "x": 3.500, "y": 5.000, "weight": 1.0 },
          { "x": 4.200, "y": 5.800, "weight": 0.7071 },
          { "x": 5.000, "y": 6.500, "weight": 1.0 }
        ]
      },
      "actions": []
    }
  ]
}
```

注意：SPIN（原地旋转）不需要单独的 Edge，体现在 Node 的 `theta` 字段中。AGV 到达节点后自动旋转到目标角度。如果需要显式控制旋转，可以在 Node 上添加 Action `spin`。

---

## 七、风险与注意事项

1. **NURBS 精度**：圆弧 → NURBS → 反算圆心，需确保浮点精度满足 task_manager 的毫米级要求
2. **MQTT 可靠性**：QoS=1 确保 Order 不丢失；State 用 QoS=0 减少开销
3. **SPIN 处理**：VDA5050 标准中 Node.theta 表示到达时的朝向，可能需要 AGV 端判断是否需要原地旋转
4. **向后兼容**：迁移期间可保留 Modbus 通道作为降级方案
5. **MQTT broker**：现有的 MQTT broker（门控）可以复用，VDA5050 topic 和门控 topic 互不干扰
