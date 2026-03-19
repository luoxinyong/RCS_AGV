# AGV 调度系统功能总览

> **标记说明：**
> - 无标记 = 基础功能（项目初始已有）
> - `[MVP新增]` = AI 智能交互 MVP 阶段新增
> - `[VDA5050新增]` = VDA5050 标准适配层新增
> - `[语音新增]` = 语音输入接口新增

## 系统架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                        前端 (React + Vite)                           │
│  Dashboard │ 设备监控 │ 地图编辑 │ 历史日志 │ AI智能交互[MVP新增]      │
│                                              ├─ 文本输入              │
│                                              └─ 语音输入 [语音新增]    │
└──────┬──────────┬──────────────────────────┬─────────────────────────┘
       │ REST API │                          │ WebSocket
       │ (Axios)  │                          │ (实时状态/任务下发)
       ▼          ▼                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      后端 (Express + TypeScript)                      │
│                                                                      │
│  路由层 ──→ 控制器 ──→ 模型层(PostgreSQL)                              │
│                    ──→ 服务层                                          │
│                        ├─ scheduler.ts    (调度核心)                   │
│                        ├─ modbus.ts       (AGV Modbus 通信)           │
│                        ├─ websocket.ts    (实时推送)                   │
│                        ├─ mqtt.ts         (MQTT 通信)                 │
│                        ├─ door.ts         (卷帘门控制)                 │
│                        ├─ ai-parser.ts    (NLP 文本解析)  [MVP新增]    │
│                        ├─ stt.ts          (语音转文本)    [语音新增]    │
│                        └─ vda5050-adapter.ts (VDA5050)   [VDA5050新增] │
└──────┬──────────┬──────────┬───────────────┬─────────────────────────┘
       │ Modbus   │ MQTT     │ MQTT          │ HTTP
       │ TCP      │          │ (VDA5050)     │ (STT)
       ▼          ▼          ▼               ▼
   AGV PLC    卷帘门      外部 Fleet      VOSK / Whisper.cpp
              控制器      Manager         语音识别服务
```

---

## 一、后端功能模块

### 1. 用户认证

| 文件 | 说明 |
|------|------|
| `server/src/routes/user.ts` | 路由：POST `/api/v1/user/login` |
| `server/src/controllers/user.ts` | 校验用户名密码，签发 JWT（HS256，7天有效期） |
| `server/src/models/user.ts` | PostgreSQL `users` 表，默认管理员 admin/300283 |

### 2. 设备管理

| 文件 | 说明 |
|------|------|
| `server/src/routes/device.ts` | 路由：GET `/device`、POST `/device/add`、POST `/device/edit` |
| `server/src/controllers/device.ts` | AGV 设备的增删改查 |
| `server/src/models/device.ts` | PostgreSQL `devices` 表（id, name, ip, port, enabled） |

每台设备对应一个 Modbus TCP 连接，ip:port 指向 AGV 的 PLC。

### 3. 调度核心（最关键的模块）

| 文件 | 说明 |
|------|------|
| `server/src/services/scheduler.ts` | 系统心脏，串联所有子系统 |
| `server/src/services/modbus.ts` | Modbus TCP 客户端，读写寄存器/线圈 |
| `server/src/utils/modbus.ts` | SubTask 二进制编码/解码，状态解析 |

**scheduler.ts 启动时做的事（`startScheduling`）：**

1. **初始化 Modbus 客户端** — 从数据库读取所有设备，为每台 AGV 创建 ModbusClient
2. **启动 WebSocket 服务** — 接收前端发来的任务指令，路由到对应 AGV 的 Modbus 下发
3. **启动 1 秒状态轮询** — 每秒从每台 AGV 读取 70 个 Input 寄存器，解析为 AGVStatus，广播给所有前端
4. **配置 MQTT 连接** — 用于卷帘门控制和 VDA5050
5. **启动卷帘门服务** — 30 分钟刷新门列表
6. **`[VDA5050新增]` 启动 VDA5050 适配器**（可通过 `VDA5050_ENABLED=false` 关闭）

**核心函数：**

- `schedulingHandler(client, type, steps: SubTask[])` — 将 SubTask[] 序列化为二进制 → 分批写入 Modbus 保持寄存器 → 发送执行指令
- `stopHandler(client)` — 暂停 → 等 300ms → 停止任务
- `getAgvStatus(client)` — 读取 Modbus Input 寄存器 → 解析位置/电量/错误/任务进度

> `[VDA5050新增]` schedulingHandler、stopHandler、agvStatus、modbusClients 从模块私有改为 **export**，供 VDA5050 适配器调用。

**SubTask 编码（每个步骤 20 个 16 位寄存器，共 40 字节）：**

| offset1 值 | 含义 | 关键参数 |
|-----------|------|---------|
| 1 | 直线行驶 | offset2=速度, offset3=方向, offset4=距离, offset6-13=坐标 |
| 2 | 弧线行驶 | 同上 + offset14-15=控制点坐标 |
| 3 | 举升/放货 | offset2=高度, offset3=速度 |
| 4 | 充电 | offset2=模式, offset3=上限 |
| 9 | 自调节取货 | offset2=速度, offset6-9=坐标 |
| 99 | 任务结束标记 | — |

### 4. 卷帘门自动控制

| 文件 | 说明 |
|------|------|
| `server/src/services/door.ts` | 门设备管理，从 cargo_size 高位解码门 ID |
| `server/src/models/door.ts` | PostgreSQL `doors` 表（ip, description） |

**工作流程（在 scheduler.ts 状态轮询中触发）：**

1. 检测到 AGV 任务步骤切换 → 检查当前步骤是否经过卷帘门区域
2. 是 → 发送 Modbus 停车指令（`writeSingleCoil(0, true)`）
3. 订阅门的 MQTT 状态（`/iot/fhjlm/pub/{doorIp}`）
4. 发送开门 MQTT 指令（`/iot/fhjlm/sub/{doorIp}`）
5. 收到门已开反馈 → 解除停车（`writeSingleCoil(0, false)`）
6. 步骤结束 → 5 秒后发送关门指令

### 5. 地图管理

| 文件 | 说明 |
|------|------|
| `server/src/routes/map.ts` | 路由：GET `/map/all`、GET `/map/find`、POST `/map/edit` |
| `server/src/controllers/map.ts` | 地图数据的读写 |
| `server/src/models/map.ts` | PostgreSQL `buildings` + `floors` 表 |

地图数据结构：
- **Building** — 编号、物理宽高（米）、原点坐标
- **Floor** — 编号、AGV 原点、**points**（点位 JSON）、**lines**（直线路径 JSON）、**paths**（弧线路径 JSON）

点位和路径数据由前端地图编辑器绘制，存为 JSON。任务下发时从这些数据计算 Dijkstra 最短路径和真实坐标。

### 6. `[MVP新增]` AI 智能交互 — 文本解析

| 文件 | 说明 | 状态 |
|------|------|------|
| `server/src/routes/ai.ts` | AI 路由定义 | `[MVP新增]` |
| `server/src/controllers/ai.ts` | 调用解析/语音服务，返回结构化结果 | `[MVP新增]` + `[语音新增]` |
| `server/src/services/ai-parser.ts` | 规则引擎：正则 + 关键词 → 结构化任务 JSON | `[MVP新增]` |

**API 端点：**

| 端点 | 方法 | 说明 | 来源 |
|------|------|------|------|
| `/api/v1/ai/parse` | POST | 文本 → 结构化任务 | `[MVP新增]` |
| `/api/v1/ai/confirm` | POST | 确认执行任务 | `[MVP新增]` |
| `/api/v1/ai/aliases` | GET | 获取点位别名映射 | `[MVP新增]` |
| `/api/v1/ai/aliases` | PUT | 更新点位别名映射 | `[MVP新增]` |
| `/api/v1/ai/voice` | POST | 上传音频 → STT → 解析 | `[语音新增]` |
| `/api/v1/ai/voice/text` | POST | 接收已转写文本 → 解析 | `[语音新增]` |
| `/api/v1/ai/voice/status` | GET | 查询 STT 后端状态 | `[语音新增]` |

**支持的指令模式（ai-parser.ts）：**

| 用户输入 | 解析结果 | 示例 |
|---------|---------|------|
| "把A的货送到B" | transport（搬运） | "把A货架的货送到B工位" → from=Q001, to=Q011 |
| "去X取货送到Y" | transport | "去1号点取货送到3号点" → from=Q001, to=Q003 |
| "从X搬到Y" | transport | "从六号点搬到七号点" → from=Q006, to=Q007 |
| "把X的货放到Y" | transport | "把六号点的货放到七号点" → from=Q006, to=Q007 |
| "去充电" | charge | → 前往 P100 充电站 |
| "暂停/继续/取消/急停" | control | → pause/resume/cancel/estop |
| "去X点" | move | "去待命点" → P200 |

**点位别名系统：** "A货架"→Q001、"充电站"→P100，支持中文数字（"六号"→6）、阿拉伯数字、Q/P 编号。

> 设计原则：回内网后只替换 `parseText` 函数体为大模型 API 调用，语音接口和任务下发代码不动。

### 7. `[语音新增]` AI 智能交互 — 语音转文本（STT）

| 文件 | 说明 |
|------|------|
| `server/src/services/stt.ts` | 可插拔 STT 服务（单例），支持多种后端 |

**三种 STT 后端（通过环境变量 `STT_BACKEND` 切换）：**

| 后端 | 说明 | 配置 |
|------|------|------|
| `vosk` | 调用本地 VOSK HTTP 服务 | `STT_VOSK_URL=http://localhost:2700` |
| `whisper` | 调用本地 Whisper.cpp HTTP 服务 | `STT_WHISPER_URL=http://localhost:8080` |
| `passthrough` | 不做 STT，由外部工具或浏览器转写（**默认**） | 无需配置 |

**四种语音接入方式：**

```
方式1: 前端麦克风按钮 → 浏览器录音 → POST /voice（音频文件）→ STT 转写 → parseText
方式2: VOSK/Whisper.cpp 独立运行 → 转写完成 → POST /voice/text（文本）→ parseText
方式3: ROS 语音识别节点 → POST /voice/text（文本）→ parseText
方式4: 前端 Web Speech API → POST /voice/text（文本）→ parseText
```

**启动 VOSK 示例：**
```bash
docker run -d -p 2700:2700 alphacep/kaldi-zh:latest
# .env 中设置：STT_BACKEND=vosk
```

**启动 Whisper.cpp 示例：**
```bash
./server -m models/ggml-base.bin --host 0.0.0.0 --port 8080
# .env 中设置：STT_BACKEND=whisper
```

**外部工具直接推文本示例：**
```bash
# VOSK/Whisper 转写完成后，直接 POST 文本
curl -X POST http://localhost:3001/api/v1/ai/voice/text \
  -H "Content-Type: application/json" \
  -d '{"text": "把A货架的货送到B工位", "source": "vosk"}'
```

### 8. `[VDA5050新增]` VDA5050 标准适配层

| 文件 | 说明 |
|------|------|
| `server/src/types/vda5050.ts` | VDA5050 v2.0 TypeScript 类型定义 |
| `server/src/services/vda5050-adapter.ts` | 适配器（单例）：MQTT ↔ Modbus 翻译 |

**做了什么：** 在现有 Modbus 调度层上方套一层 VDA5050 标准 MQTT 接口，外部系统发标准 JSON，适配器翻译成内部 SubTask 二进制下发。

| 方向 | MQTT 主题 | 功能 |
|------|----------|------|
| 接收 | `uagv/v2/myagv/{agvId}/order` | VDA5050 Order → SubTask[] → Modbus 下发 |
| 接收 | `uagv/v2/myagv/{agvId}/instantActions` | cancelOrder/startPause/stopPause → Modbus 控制 |
| 发布 | `uagv/v2/myagv/{agvId}/state` | 每秒：AGVStatus → VDA5050 State JSON |
| 发布 | `uagv/v2/myagv/{agvId}/connection` | 连接状态（retained） |

**翻译规则：**

| VDA5050 元素 | SubTask offset1 | 说明 |
|-------------|-----------------|------|
| Edge（节点间移动） | 1（直线行驶） | 从 nodePosition 计算坐标、距离、方向 |
| Action `pick` | 9（自调节取货） | 节点位置作为取货坐标 |
| Action `drop` | 3（举升放货） | 节点位置作为放货坐标 |
| Action `charge` | 4（充电） | 节点位置作为充电坐标 |
| Order 结束 | 99（任务完成） | 自动添加 |

**单位转换：** 米↔毫米、弧度↔0.1度、m/s↔mm/s。

可通过环境变量 `VDA5050_ENABLED=false` 关闭。

### 9. 任务/告警日志

| 文件 | 说明 |
|------|------|
| `server/src/routes/tasks.ts` + `controllers/tasks.ts` | 任务日志查询（分页、筛选、排序） |
| `server/src/routes/alarms.ts` + `controllers/alarms.ts` | 告警日志查询 |
| `server/src/models/tasks.ts` | PostgreSQL `tasks` 表 |
| `server/src/models/alarms.ts` | PostgreSQL `alarms` 表 |

任务日志在 `schedulingHandler`/`stopHandler` 执行后自动写入。告警日志在状态轮询检测到错误时自动写入。

### 10. 请求审计日志

| 文件 | 说明 |
|------|------|
| `server/src/middlewares/logger.ts` | 拦截所有 HTTP 请求，记录方法/路径/状态码/耗时/请求体/响应体 |
| `server/src/models/log.ts` | PostgreSQL `logs` 表 |

### 11. 数据库自动初始化

| 文件 | 说明 |
|------|------|
| `server/src/services/postgres.ts` | 连接池 + 启动时自动建表（8 张表） |

**自动创建的表：** users, logs, devices, tasks, alarms, buildings, floors, doors

---

## 二、前端功能模块

### 1. 登录页（`/login`）

| 文件 | 说明 |
|------|------|
| `client/src/pages/Login.tsx` | 用户名密码登录，JWT 存入 localStorage |

### 2. 总览仪表盘（`/`）

| 文件 | 说明 |
|------|------|
| `client/src/pages/Overview.tsx` | 设备统计、实时 WebSocket 状态显示 |

### 3. 设备监控（`/monitor/status`）

| 文件 | 说明 |
|------|------|
| `client/src/pages/monitor/Status.tsx` | AGV 状态卡片：位置、电量、充电状态、速度 |

通过 WebSocket 每秒接收 AGV 状态更新。

### 4. 地图编辑器（`/map/list` + `/map/editor`）

| 文件 | 说明 |
|------|------|
| `client/src/pages/map/List.tsx` | 选择建筑物/楼层 |
| `client/src/pages/map/editor.tsx` | 核心：Fabric.js 画布编辑器 + 任务下发 |
| `client/src/components/map-editor/` | 工具栏、右侧属性栏、状态栏、帮助面板 |
| `client/src/lib/map-editor/index.ts` | 路径算法：Dijkstra 最短路径、SubTask 生成 |
| `client/src/hooks/map-editor/useCanvas.ts` | 画布交互逻辑 |

**地图编辑器功能：**
- 绘制点位（取放货点 Q / 路径点 P）
- 绘制直线路径、弧线路径
- 设置点位属性（障碍区域、旋转角度、RFID EPC）
- 设置路径属性（速度、距离、方向、卷帘门关联）
- **任务下发**：选择 AGV → 选择起终点 → Dijkstra 寻路 → 生成 SubTask[] → WebSocket 发送

### 5. 历史日志（`/history/task` + `/history/alarm`）

| 文件 | 说明 |
|------|------|
| `client/src/pages/history/Task.tsx` | 任务历史表格，支持分页、按类型筛选、排序 |
| `client/src/pages/history/Alarm.tsx` | 告警历史表格，显示错误码 |

### 6. `[MVP新增]` AI 智能交互（`/ai/chat`）

| 文件 | 说明 | 状态 |
|------|------|------|
| `client/src/pages/ai/ChatPanel.tsx` | 聊天界面 + 语音输入 | `[MVP新增]` + `[语音新增]` |

**完整流程（文本）：**
1. 用户输入自然语言（如 "把A货架的货送到B工位"）
2. 调用后端 `/api/v1/ai/parse` 得到结构化任务
3. 前端展示解析结果（起终点、任务类型、置信度、步骤预览）
4. 用户选择 AGV、确认任务
5. 前端加载地图数据 → Dijkstra 寻路 → 计算真实坐标 → 生成完整 SubTask[]
6. 通过 WebSocket 发送到后端 → schedulingHandler → Modbus 下发到 AGV

**`[语音新增]` 完整流程（语音）：**
1. 用户按住麦克风按钮说话
2. 浏览器 MediaRecorder 录音（webm 格式）
3. 松开按钮 → 音频上传到 `POST /api/v1/ai/voice`
4. 后端 STT 转写 → 文本解析 → 返回结构化任务
5. 后续流程同文本（展示结果 → 确认 → 下发）

### 7. 侧边栏导航

| 文件 | 说明 |
|------|------|
| `client/src/components/Sidebar.tsx` | 可折叠导航：总览、设备监控、地图管理、历史日志、AI交互 |

### 8. 公共基础设施

| 文件 | 说明 |
|------|------|
| `client/src/network/axios.ts` | Axios 实例，自动注入 JWT，响应拦截器 |
| `client/src/network/api.ts` | 所有 REST API 封装 |
| `client/src/hooks/useApi.ts` | 通用异步请求 Hook（loading/data/error 状态管理） |
| `client/src/hooks/useWebSocket.ts` | WebSocket Hook（自动重连、消息解析） |
| `client/src/config/index.ts` | 服务器地址、AGV 运动参数、localStorage key |
| `client/src/routes/index.tsx` | React Router v6 路由配置 |

---

## 三、通信协议总览

| 协议 | 用途 | 端口 | 数据格式 | 来源 |
|------|------|------|---------|------|
| **HTTP REST** | 前端 ↔ 后端 API | 3001 | JSON | 基础 |
| **WebSocket** | 实时状态推送 + 任务下发 | 3001 | JSON | 基础 |
| **Modbus TCP** | 后端 ↔ AGV PLC | 设备自定义 | 二进制寄存器 | 基础 |
| **MQTT** | 卷帘门控制 | Broker 自定义 | JSON | 基础 |
| **MQTT (VDA5050)** | 外部调度 ↔ 适配器 | 同上 | VDA5050 JSON | `[VDA5050新增]` |
| **HTTP (STT)** | 后端 → VOSK/Whisper 服务 | 2700/8080 | 音频/JSON | `[语音新增]` |
| **JWT** | 用户认证 | — | HS256 Token | 基础 |

---

## 四、数据库表

| 表名 | 说明 | 自动写入 |
|------|------|---------|
| `users` | 用户账号（默认 admin/300283） | 首次启动 |
| `devices` | AGV 设备列表（ip, port） | 手动添加 |
| `buildings` | 建筑物定义（宽高、原点） | 地图编辑器 |
| `floors` | 楼层地图（点位、路径 JSON） | 地图编辑器 |
| `doors` | 卷帘门（ip） | 手动添加 |
| `tasks` | 任务执行日志 | 调度器自动 |
| `alarms` | 错误/告警日志 | 状态轮询自动 |
| `logs` | HTTP 请求审计日志 | 中间件自动 |

---

## 五、任务下发通路

```
通路1（前端文本/地图编辑器）:
  前端 ChatPanel/MapEditor → WebSocket → scheduler.sendMessageToModbus
    → schedulingHandler(client, type, SubTask[]) → Modbus TCP → AGV PLC

通路2 [语音新增]（前端语音）:
  前端麦克风录音 → POST /voice → STT 转写 → parseText → 返回结构化任务
    → 用户确认 → 同通路1 的 WebSocket 下发

通路3 [语音新增]（外部 STT 工具）:
  VOSK/Whisper/ROS语音节点 → POST /voice/text → parseText → 返回结构化任务
    → 外部系统可根据返回的 task 自行调用 WebSocket 下发

通路4 [VDA5050新增]（外部 Fleet Manager）:
  Fleet Manager → MQTT Order → vda5050-adapter.handleOrder
    → translateOrderToSubTasks → schedulingHandler(client, "vda5050", SubTask[])
    → Modbus TCP → AGV PLC
```

所有通路最终都汇入 `schedulingHandler` → Modbus 写寄存器。互不影响，可同时运行。

---

## 六、环境变量配置总览

| 变量 | 默认值 | 说明 | 来源 |
|------|--------|------|------|
| `PORT` | 3001 | 后端端口 | 基础 |
| `JWT_SECRET` | — | JWT 签名密钥 | 基础 |
| `MQTT_HOST` | — | MQTT Broker 地址 | 基础 |
| `MQTT_PORT` | — | MQTT Broker 端口 | 基础 |
| `PG_HOST` | — | PostgreSQL 地址 | 基础 |
| `PG_PORT` | — | PostgreSQL 端口 | 基础 |
| `PG_USER` | — | PostgreSQL 用户名 | 基础 |
| `PG_PASSWORD` | — | PostgreSQL 密码 | 基础 |
| `PG_DATABASE` | — | PostgreSQL 数据库名 | 基础 |
| `VDA5050_ENABLED` | `true` | 是否启用 VDA5050 适配层 | `[VDA5050新增]` |
| `VDA5050_MANUFACTURER` | `myagv` | VDA5050 MQTT 主题中的制造商标识 | `[VDA5050新增]` |
| `STT_BACKEND` | `passthrough` | STT 后端类型：vosk / whisper / passthrough | `[语音新增]` |
| `STT_VOSK_URL` | `http://localhost:2700` | VOSK HTTP 服务地址 | `[语音新增]` |
| `STT_WHISPER_URL` | `http://localhost:8080` | Whisper.cpp HTTP 服务地址 | `[语音新增]` |

---

## 七、文件变更追踪

### 基础功能文件（项目初始）

```
server/src/
├── index.ts                          # Express 入口
├── config/app.ts                     # 配置
├── routes/user.ts, device.ts, tasks.ts, alarms.ts, map.ts, door.ts
├── controllers/user.ts, device.ts, tasks.ts, alarms.ts, map.ts, door.ts
├── services/scheduler.ts, modbus.ts, websocket.ts, mqtt.ts, door.ts, postgres.ts
├── models/user.ts, device.ts, tasks.ts, alarms.ts, map.ts, door.ts, log.ts
├── middlewares/error.ts, logger.ts, validate.ts
├── utils/modbus.ts
├── types/device.ts
└── validators/index.ts

client/src/
├── main.tsx, routes/index.tsx, config/index.ts
├── pages/Login.tsx, Overview.tsx, Settings.tsx
├── pages/monitor/Status.tsx
├── pages/map/List.tsx, editor.tsx
├── pages/history/Task.tsx, Alarm.tsx
├── components/Sidebar.tsx, Header.tsx, map-editor/*
├── hooks/useApi.ts, useWebSocket.ts, map-editor/useCanvas.ts
├── network/axios.ts, api.ts
├── lib/map-editor/index.ts
└── types/*
```

### `[MVP新增]` AI 智能交互

```
server/src/
├── routes/ai.ts                      # 新建：AI 路由
├── controllers/ai.ts                 # 新建：AI 控制器
└── services/ai-parser.ts             # 新建：规则解析引擎

client/src/
└── pages/ai/ChatPanel.tsx             # 新建：智能交互面板
```

改动的已有文件：
- `server/src/index.ts` — 注册 ai 路由，JWT 白名单加入 `/ai/*`
- `client/src/routes/index.tsx` — 添加 `/ai/chat` 路由
- `client/src/components/Sidebar.tsx` — 导航添加 "AI交互" 菜单项

### `[VDA5050新增]` VDA5050 标准适配层

```
server/src/
├── types/vda5050.ts                   # 新建：VDA5050 类型定义
└── services/vda5050-adapter.ts        # 新建：VDA5050 适配器
```

改动的已有文件：
- `server/src/services/scheduler.ts` — 导出 4 个内部变量/函数，末尾添加 VDA5050 启动
- `server/src/config/app.ts` — 新增 `VDA5050_ENABLED`、`VDA5050_MANUFACTURER`

### `[语音新增]` 语音输入接口

```
server/src/
└── services/stt.ts                    # 新建：STT 语音转文本服务
```

改动的已有文件：
- `server/src/routes/ai.ts` — 新增 3 个语音端点（voice, voice/text, voice/status）
- `server/src/controllers/ai.ts` — 新增 3 个语音处理函数
- `server/src/config/app.ts` — 新增 `STT_BACKEND`、`STT_VOSK_URL`、`STT_WHISPER_URL`
- `client/src/pages/ai/ChatPanel.tsx` — 新增麦克风按钮（按住录音）
- `server/package.json` — 新增 `multer` 依赖（音频文件上传）
