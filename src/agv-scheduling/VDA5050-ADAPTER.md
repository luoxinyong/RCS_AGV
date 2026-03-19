# VDA5050 适配层技术文档

## 1. 背景与目标

当前 AGV 调度系统使用私有 Modbus TCP 二进制协议与 AGV 通信（SubTask offset1~20），调度层和 AGV 强耦合。

**VDA5050** 是德国汽车工业协会制定的 AGV/AMR 通信标准（v2.0），基于 MQTT + JSON，是行业通用协议。加入 VDA5050 标准层的目的：

- 上层系统（Fleet Manager、MES、WMS）说标准协议即可对接
- 下层保持现有 Modbus 不动，不影响已有功能
- 便于对接不同厂商的 AGV 或调度系统

## 2. 架构设计

```
外部调度 / Fleet Manager
      │
      │  MQTT (VDA5050 JSON)
      │
      │  订阅: uagv/v2/myagv/{agvId}/order          (接收任务)
      │  订阅: uagv/v2/myagv/{agvId}/instantActions  (接收即时指令)
      │  发布: uagv/v2/myagv/{agvId}/state            (状态上报)
      │  发布: uagv/v2/myagv/{agvId}/connection        (连接状态)
      │
      ▼
┌─────────────────────────────┐
│   vda5050-adapter.ts        │  ← 新增：适配器（单例）
│                             │
│   Order → SubTask[] 翻译    │
│   AGVStatus → State 翻译    │
│   InstantActions 处理       │
└─────────────┬───────────────┘
              │ 调用已有函数
              ▼
┌─────────────────────────────┐
│   scheduler.ts              │  ← 微改：导出 4 个内部变量
│                             │
│   schedulingHandler()       │  任务下发
│   stopHandler()             │  任务停止
│   agvStatus Map             │  AGV 状态缓存
│   modbusClients[]           │  Modbus 客户端列表
└─────────────┬───────────────┘
              │ Modbus TCP
              ▼
┌─────────────────────────────┐
│   rcs_interaction (C++)     │  ← 不动
│   task_manager (C++)        │  ← 不动
│   AGV PLC                   │  ← 不动
└─────────────────────────────┘
```

**两条下发通路并存，互不影响：**

| 通路 | 入口 | 流程 |
|------|------|------|
| 前端/AI | WebSocket | 前端 → WebSocket → sendMessageToModbus → schedulingHandler → Modbus |
| VDA5050 | MQTT | Fleet Manager → MQTT Order → vda5050-adapter → schedulingHandler → Modbus |

## 3. 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `server/src/types/vda5050.ts` | **新建** | VDA5050 v2.0 TypeScript 类型定义 |
| `server/src/services/vda5050-adapter.ts` | **新建** | 核心适配器，单例模式 |
| `server/src/services/scheduler.ts` | **微改** | 导出 4 个内部变量/函数，添加 VDA5050 启动调用 |
| `server/src/config/app.ts` | **微改** | 新增 VDA5050_ENABLED、VDA5050_MANUFACTURER 配置 |

不改动：`rcs_interaction`、`task_manager`、`plc_interaction`、前端代码、现有 WebSocket 链路。

## 4. VDA5050 协议核心概念

### 4.1 MQTT 主题规范

```
uagv/v2/{manufacturer}/{serialNumber}/{topic}
```

本系统中 `manufacturer = "myagv"`（可通过环境变量修改），`serialNumber` 对应 AGV 设备 ID。

### 4.2 四种核心消息

| 消息类型 | 方向 | 说明 |
|---------|------|------|
| **Order** | 外部 → 适配器 | 包含 Node-Edge 图结构，描述 AGV 要走的路径和要执行的动作 |
| **InstantActions** | 外部 → 适配器 | 即时指令，如取消任务、暂停、恢复 |
| **State** | 适配器 → 外部 | AGV 实时状态（位置、电量、任务进度、错误等），每秒发布 |
| **Connection** | 适配器 → 外部 | 连接状态（ONLINE/OFFLINE），retained 消息 |

### 4.3 Order 结构示例

```json
{
  "orderId": "order-001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "1", "sequenceId": 0, "released": true,
      "nodePosition": { "x": 1.0, "y": 2.0, "mapId": "1" },
      "actions": []
    },
    {
      "nodeId": "2", "sequenceId": 2, "released": true,
      "nodePosition": { "x": 3.0, "y": 4.0, "mapId": "1" },
      "actions": [{ "actionType": "pick", "actionId": "a1", "blockingType": "HARD" }]
    },
    {
      "nodeId": "3", "sequenceId": 4, "released": true,
      "nodePosition": { "x": 5.0, "y": 6.0, "mapId": "1" },
      "actions": [{ "actionType": "drop", "actionId": "a2", "blockingType": "HARD" }]
    }
  ],
  "edges": [
    {
      "edgeId": "e1", "sequenceId": 1, "released": true,
      "startNodeId": "1", "endNodeId": "2", "maxSpeed": 0.5, "actions": []
    },
    {
      "edgeId": "e2", "sequenceId": 3, "released": true,
      "startNodeId": "2", "endNodeId": "3", "maxSpeed": 0.5, "actions": []
    }
  ]
}
```

上述 Order 含义：从节点1移动到节点2 → 在节点2取货 → 从节点2移动到节点3 → 在节点3放货。

## 5. 翻译规则

### 5.1 Order → SubTask[] 翻译

适配器将 VDA5050 的 Node-Edge 图结构翻译为内部 SubTask 序列：

| VDA5050 元素 | SubTask offset1 | 说明 |
|-------------|-----------------|------|
| Edge（节点间移动） | 1（直线行驶） | 从 nodePosition 计算坐标、距离、方向 |
| Action `pick` | 9（自调节取货） | 节点位置作为取货坐标 |
| Action `drop` | 3（举升放货） | 节点位置作为放货坐标 |
| Action `charge` | 4（充电） | 节点位置作为充电坐标 |
| Order 结束 | 99（任务完成） | 自动添加 |

### 5.2 单位转换

| VDA5050 | 内部系统 | 转换 |
|---------|---------|------|
| 位置：米（float） | 毫米（int） | × 1000 |
| 角度：弧度（float） | 0.1度（int） | × 180/π × 10 |
| 速度：m/s（float） | mm/s（int） | × 1000 |

### 5.3 坐标编码

VDA5050 坐标（米）→ 毫米（32位整数）→ 拆分为两个 16 位整数写入 SubTask 的 offset6~13：

```
offset6  = x_low   (低16位)
offset7  = x_high  (高16位)
offset8  = y_low
offset9  = y_high
offset10 = bx_low  (终点x)
offset11 = bx_high
offset12 = by_low
offset13 = by_high
```

### 5.4 AGVStatus → VDA5050 State 翻译

适配器从 scheduler 的 `agvStatus` Map 读取（每秒由 Modbus 轮询更新），无需额外通信：

| AGVStatus 字段 | VDA5050 State 字段 | 转换 |
|---------------|-------------------|------|
| px, py（mm） | agvPosition.x, y | ÷ 1000 → 米 |
| pa（度） | agvPosition.theta | × π/180 → 弧度 |
| traction_speed（mm/s） | velocity.vx | ÷ 1000 → m/s |
| energy_level（0~100） | batteryState.batteryCharge | 直接映射 |
| status_charge（1=充电中） | batteryState.charging | 1 → true |
| err1~5, warnings | errors[] | 非零则加入错误列表 |
| traction_is_parked + speed | driving | 未驻车且速度>0 → true |
| step_en_pause | paused | 1 → true |
| cargo_loaded | loads[] | 非零则包含一个 load |
| tr_step_index, tr_cplt_percent | nodeStates/edgeStates | 根据步骤进度更新 |

### 5.5 InstantActions 处理

| VDA5050 ActionType | 内部操作 |
|-------------------|---------|
| `cancelOrder` | 调用 `stopHandler(client)` 暂停+停止任务 |
| `startPause` | `client.writeSingleCoil(0, true)` 写入停车 |
| `stopPause` | `client.writeSingleCoil(0, false)` 解除停车 |

## 6. 配置

在 `.env` 文件或环境变量中配置：

```bash
# 是否启用 VDA5050 适配层（默认启用）
VDA5050_ENABLED=true

# VDA5050 制造商标识（用于 MQTT 主题）
VDA5050_MANUFACTURER=myagv

# MQTT Broker 地址（VDA5050 复用现有 MQTT 连接）
MQTT_HOST=192.168.1.100
MQTT_PORT=1883
```

## 7. 验证方法

### 7.1 启动确认

重启后端，控制台应输出：
```
✅ VDA5050 适配器已启动，制造商: myagv，AGV数量: 1
```

MQTT 连接后应输出：
```
📡 VDA5050: MQTT 已连接，注册订阅...
📌 Subscribed to topic: uagv/v2/myagv/1/order
📌 Subscribed to topic: uagv/v2/myagv/1/instantActions
```

### 7.2 监听 State 上报

```bash
mosquitto_sub -h {MQTT_HOST} -t "uagv/v2/myagv/+/state" -v
```

应每秒收到 JSON 格式的 AGV 状态。

### 7.3 发送测试 Order

```bash
mosquitto_pub -h {MQTT_HOST} -t "uagv/v2/myagv/1/order" -m '{
  "headerId": 1,
  "timestamp": "2026-03-19T14:00:00Z",
  "version": "2.0.0",
  "manufacturer": "myagv",
  "serialNumber": "1",
  "orderId": "test-001",
  "orderUpdateId": 0,
  "nodes": [
    {"nodeId": "1", "sequenceId": 0, "released": true,
     "nodePosition": {"x": 1.0, "y": 2.0, "mapId": "1"}, "actions": []},
    {"nodeId": "2", "sequenceId": 2, "released": true,
     "nodePosition": {"x": 3.0, "y": 4.0, "mapId": "1"}, "actions": []}
  ],
  "edges": [
    {"edgeId": "e1", "sequenceId": 1, "released": true,
     "startNodeId": "1", "endNodeId": "2", "maxSpeed": 0.5, "actions": []}
  ]
}'
```

后端应输出：
```
📥 VDA5050 收到 Order: agv=1, orderId=test-001, updateId=0
✅ VDA5050: Order test-001 已下发到 AGV 1，共 2 步
```

### 7.4 发送 InstantAction

```bash
mosquitto_pub -h {MQTT_HOST} -t "uagv/v2/myagv/1/instantActions" -m '{
  "headerId": 1,
  "timestamp": "2026-03-19T14:01:00Z",
  "version": "2.0.0",
  "manufacturer": "myagv",
  "serialNumber": "1",
  "actions": [{"actionType": "cancelOrder", "actionId": "ia1", "blockingType": "HARD"}]
}'
```

### 7.5 关闭适配层

设置环境变量 `VDA5050_ENABLED=false` 重启即可关闭，不影响其他功能。

## 8. MVP 范围限制

当前为 MVP 实现，存在以下限制：

1. **仅支持直线移动**（offset1=1），不支持弧线（offset1=2）
2. **不支持 Order horizon 扩展**：收到新 orderUpdateId 时直接重新下发整个任务
3. **节点/边进度追踪为粗略映射**：基于 SubTask 步骤索引的比例推算，非精确映射
4. **Action 类型有限**：仅支持 pick / drop / charge，其他 action 类型会跳过
5. **不支持多地图**：mapId 固定为 "1"
6. **不做 VDA5050 协议校验**：信任外部发送的 Order 格式

## 9. 后续扩展方向

- 支持弧线 Edge（offset1=2），需要额外的控制点坐标
- Order horizon 管理（base/horizon 节点划分）
- 精确的 Node/Edge 进度追踪（结合点位 ID 匹配）
- 完整的 VDA5050 Action 支持（如 initPosition、finePositioning 等）
- 多地图支持
- MQTT topic 路由优化（避免全局 callback 过滤）
