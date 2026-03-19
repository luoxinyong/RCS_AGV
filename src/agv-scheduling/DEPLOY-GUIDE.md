# AGV 调度系统 — 内网离线部署指南

## 1. 部署包内容

```
agv-scheduling-offline/
├── client/dist/          # 前端静态页面（19MB）
├── server/dist/          # 后端 API 编译产物
├── server/node_modules/  # 后端依赖（77MB）
├── server/.env.template  # 环境变量模板
├── node/                 # Node.js v22.22.1 便携版（205MB）
├── whisper/              # Whisper.cpp STT 语音转文本
│   ├── bin/              #   whisper-server、whisper-cli
│   ├── lib/              #   动态链接库
│   └── models/           #   ggml-small.bin 模型（466MB）
├── start-all.sh          # 一键启动
└── stop-all.sh           # 一键停止
```

总大小约 512MB（压缩包），解压后约 770MB。

---

## 2. 前置条件

部署机器需要满足：

| 条件 | 要求 | 检查命令 |
|------|------|----------|
| 系统 | x86_64 Linux（Ubuntu 18.04+/CentOS 7+） | `uname -m` 应输出 `x86_64` |
| glibc | ≥ 2.17 | `ldd --version` |
| PostgreSQL | 已安装并运行 | `psql --version` |
| MQTT Broker | mosquitto 或其他 MQTT 服务 | `mosquitto -h` 或 `systemctl status mosquitto` |
| 端口 | 3001、5173、8080 未被占用 | `ss -tlnp \| grep -E '3001\|5173\|8080'` |

> Node.js 和 Whisper 已包含在部署包中，**不需要**单独安装。

### 2.1 安装 PostgreSQL（如果没有）

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y postgresql postgresql-contrib

# CentOS/RHEL
sudo yum install -y postgresql-server postgresql-contrib
sudo postgresql-setup initdb

# 启动
sudo systemctl start postgresql
sudo systemctl enable postgresql
```

### 2.2 安装 MQTT Broker（如果没有）

```bash
# Ubuntu/Debian
sudo apt-get install -y mosquitto mosquitto-clients

# CentOS/RHEL
sudo yum install -y mosquitto

# 启动
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

---

## 3. 部署步骤

### 3.1 解压

```bash
# 将 agv-scheduling-offline.tar.gz 拷贝到目标机器后
tar xzf agv-scheduling-offline.tar.gz
cd agv-scheduling-offline
```

### 3.2 创建数据库

```bash
# 切换到 postgres 用户
sudo -u postgres psql

# 在 psql 中执行
CREATE USER postgres WITH PASSWORD '你的密码';
CREATE DATABASE agv OWNER postgres;
\q
```

> 如果 postgres 用户已存在，只需创建 agv 数据库即可。系统首次启动时会自动建表。

### 3.3 配置环境变量

```bash
cp server/.env.template server/.env
```

编辑 `server/.env`，**必须修改**以下项：

```bash
# 数据库（改成实际值）
PG_HOST=localhost
PG_PORT=5432
PG_USER=postgres
PG_PASSWORD=你的数据库密码
PG_DATABASE=agv

# MQTT（改成实际 MQTT 地址）
MQTT_HOST=127.0.0.1
MQTT_PORT=1883
```

其他项按需修改：

```bash
PORT=3001                         # 后端端口
JWT_SECRET=agv-dev-secret-key-2024  # JWT 密钥（生产环境建议更换）
STT_BACKEND=whisper               # 语音后端（whisper/passthrough）
STT_WHISPER_URL=http://localhost:8080
VDA5050_ENABLED=false             # VDA5050 适配层开关
```

### 3.4 启动

```bash
./start-all.sh
```

正常输出：

```
=========================================
  AGV Scheduling System
  Node: v22.22.1
=========================================
[1/3] 启动 Whisper STT 服务 (端口 8080)...
    PID: 12345
[2/3] 启动后端 API 服务 (端口 3001)...
    PID: 12346
[3/3] 启动前端服务 (端口 5173)...
    PID: 12347

=========================================
  全部启动完成!

  前端:    http://localhost:5173
  后端API: http://localhost:3001
  Whisper: http://localhost:8080

  日志目录: .../logs/
  停止服务: ./stop-all.sh
=========================================
```

### 3.5 停止

```bash
./stop-all.sh
```

---

## 4. 验证测试

按以下顺序逐步验证，确保每一步都通过后再进行下一步。

### 4.1 Whisper STT 服务

```bash
# 检查服务是否存活
curl http://localhost:8080

# 应返回 HTML 页面（Whisper.cpp Server 测试界面）
```

如果有 wav/mp3 音频文件可以直接测试转写：

```bash
curl -X POST http://localhost:8080/inference \
  -F "file=@你的音频.wav" \
  -F "language=zh" \
  -F "response_format=json"

# 预期返回: {"text":"识别到的中文文本\n"}
```

### 4.2 后端 API 服务

```bash
# 健康检查 — STT 状态
curl http://localhost:3001/api/v1/ai/voice/status

# 预期返回:
# {"success":true,"backend":"whisper","healthy":true}
```

```bash
# 文本解析测试
curl -X POST http://localhost:3001/api/v1/ai/parse \
  -H "Content-Type: application/json" \
  -d '{"text":"到五号点去"}'

# 预期返回:
# {"success":true,"task":{"taskType":"move","to":"P005",...}}
```

```bash
# 语音 → 文本 → 解析（完整链路）
# 需要准备一段中文语音文件
curl -X POST http://localhost:3001/api/v1/ai/voice \
  -F "audio=@你的音频.wav" \
  -F "format=wav" \
  -F "language=zh"

# 预期返回:
# {"success":true,"task":{...},"stt":{"text":"识别结果","backend":"whisper",...}}
```

```bash
# 外部 STT 推送文本接口（如果用外部工具转写好了文本）
curl -X POST http://localhost:3001/api/v1/ai/voice/text \
  -H "Content-Type: application/json" \
  -d '{"text":"去六号点","source":"external"}'

# 预期返回:
# {"success":true,"task":{"taskType":"move","to":"P006",...},"stt":{"text":"去六号点",...}}
```

### 4.3 前端页面

浏览器打开 `http://<部署机器IP>:5173`

1. 应看到登录页面
2. 登录后进入调度主界面
3. 进入 AI 交互页面，输入文字指令测试（如"到五号点去"）
4. 点击麦克风按钮（按住说话）测试语音输入

### 4.4 AGV Modbus 通信

```bash
# 查看后端日志，确认 Modbus 连接
tail -f logs/server.log

# 如果有 AGV 在线，应看到类似:
# ✅ AGV 1 已连接 (192.168.x.x:502)
# 如果 AGV 未开机，会看到连接重试日志（正常）
```

### 4.5 完整流程测试

1. 确保至少一台 AGV 在线（Modbus 可达）
2. 前端选择该 AGV
3. AI 面板输入"到五号点去"
4. 确认解析结果，点击"确认下发"
5. 观察 AGV 是否开始执行任务

---

## 5. 日志排查

所有日志在 `logs/` 目录下：

| 文件 | 内容 |
|------|------|
| `logs/whisper.log` | Whisper STT 服务日志（模型加载、转写请求） |
| `logs/server.log` | 后端日志（API 请求、Modbus 通信、数据库） |
| `logs/frontend.log` | 前端静态服务日志 |

常见问题：

| 现象 | 原因 | 解决 |
|------|------|------|
| `voice/status` 返回 `healthy:false` | Whisper 服务未启动或端口不对 | 检查 whisper.log，确认 8080 端口 |
| 后端启动报数据库错误 | PG 配置错误或数据库不存在 | 检查 .env 中 PG_ 开头的配置项 |
| 语音识别结果不准 | base 模型精度有限 | 确认使用的是 ggml-small.bin |
| 前端页面打不开 | 5173 端口未启动 | 检查 frontend.log |
| 前端能打开但 API 报错 | 前端请求的后端地址不对 | 检查前端 config 中的 API 地址配置 |

---

## 6. 前端 API 地址配置

前端连接后端的地址在构建时确定。如果后端不在 `localhost:3001`，需要修改前端配置后重新构建。

在开发机上修改 `client/src/config/index.ts` 中的服务器地址，然后重新构建：

```bash
cd client
npm run build
# 将新的 dist/ 目录复制到部署包的 client/dist/ 中
```

或者使用 nginx 反向代理统一入口（推荐生产方式）。

---

## 7. 使用 Nginx 部署（推荐）

生产环境建议用 nginx 替代内置的静态文件服务：

```nginx
server {
    listen 80;
    server_name your-domain.com;

    # 前端
    location / {
        root /path/to/agv-scheduling-offline/client/dist;
        try_files $uri $uri/ /index.html;
    }

    # 后端 API 代理
    location /api/ {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
    }

    # WebSocket 代理
    location /ws {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

---

## 8. 端口总览

| 端口 | 服务 | 说明 |
|------|------|------|
| 5173 | 前端页面 | 浏览器访问入口 |
| 3001 | 后端 API + WebSocket | REST API 和实时通信 |
| 8080 | Whisper STT | 语音转文本服务 |
| 5432 | PostgreSQL | 数据库（需预装） |
| 1883 | MQTT Broker | 设备通信（需预装） |

---

## 9. 生成测试音频（可选）

如果内网没有录音设备，可以用以下方式生成测试音频（需要外网先生成后拷入）：

```bash
# 在外网机器上（需要 pip install gTTS）
python3 -c "
from gtts import gTTS
for text, name in [
    ('到五号点去', 'goto5'),
    ('去六号点', 'goto6'),
    ('暂停', 'pause'),
    ('去充电', 'charge'),
    ('把一号货架的货送到二号工位', 'transport'),
]:
    gTTS(text, lang='zh-cn').save(f'test_{name}.mp3')
    print(f'生成: test_{name}.mp3 ({text})')
"

# 将生成的 mp3 文件拷贝到内网后测试
curl -X POST http://localhost:3001/api/v1/ai/voice \
  -F "audio=@test_goto5.mp3" -F "format=mp3" -F "language=zh"
```
