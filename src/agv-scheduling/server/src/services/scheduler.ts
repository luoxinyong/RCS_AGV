import { Server } from "http";
import WebSocket from "ws";
import MqttClient from "./mqtt";
import DoorService from "./door";
import ModbusClient from "./modbus";
import WebSocketClient from "./websocket";
import APP_CONFIG from "../config/app";
import { insertTask } from "../models/tasks";
import { insertAlarm } from "../models/alarms";
import { queryDevices } from "../models/device";
import {
  covertToTaskBuffer,
  parseStatusBuffer,
  parseTaskHexString,
  TaskType,
  type SubTask,
} from "../utils/modbus";
import type { AGVStatus } from "../types/device";
import VDA5050Adapter from "./vda5050-adapter";

// 保留agv最新的一条记录，用于根据当前状态下发指令
export const agvStatus = new Map<String, AGVStatus>();

// modbus客户端列表（在startScheduling中初始化）
export let modbusClients: ModbusClient[] = [];

export async function startScheduling(server: Server) {
  // 获取所有设备
  const devices = await queryDevices();

  // 1.初始化modbus通信
  modbusClients = devices.map(
    (e) => new ModbusClient(e.id.toString(), e.ip, e.port)
  );

  // 2-1.启动websocket
  WebSocketClient.getInstance().initialize(server, {
    onMessage: async (ws, msg) => {
      try {
        const json = JSON.parse(msg.toString());
        const client = modbusClients.find((e) => e.id === json.data.agvId);
        if (!client) {
          throw new Error("未找到对应的AGV设备编号");
        }
        sendMessageToModbus({
          ws,
          client,
          type: json.type ?? "unknown",
          steps: json.data.message,
        });
      } catch (error: any) {
        WebSocketClient.getInstance().sendData(ws, {
          type: "error",
          data: {
            title: error.message ?? "错误的消息格式",
            message: msg.toString(),
          },
        });
      }
    },
  });

  // 2-2.定时读取所有agv状态，并广播数据
  async function updateLoop() {
    WebSocketClient.getInstance().broadcastData({
      type: "update",
      data: await Promise.all(modbusClients.map(getAgvStatus)),
    });
    setTimeout(updateLoop, 1000);
  }
  updateLoop();

  // 3.配置 MQTT 单例客户端连接，用于监听卷帘门状态，以及发送开关门指令
  MqttClient.getInstance().configure({
    brokerUrl: `mqtt://${APP_CONFIG.MQTT_HOST}:${APP_CONFIG.MQTT_PORT}`,
    options: { reconnectPeriod: 3000, connectTimeout: 10000 },
    onConnected: () => {
      // MQTT 连接/重连后，VDA5050 适配器注册订阅
      if (APP_CONFIG.VDA5050_ENABLED) {
        VDA5050Adapter.getInstance().onMqttConnected();
      }
    },
  });

  // 4.启动卷帘门服务，30分钟
  DoorService.getInstance().start(30 * 60 * 1000);

  // 5.启动VDA5050适配层
  if (APP_CONFIG.VDA5050_ENABLED) {
    VDA5050Adapter.getInstance().start();
  }
}

async function getAgvStatus(client: ModbusClient) {
  let status = agvStatus.get(client.id);
  let isOnline = false;
  try {
    const startDate = new Date();
    console.log("读取modbus: ", startDate.toTimeString());
    const result = await client.readRegisters("input", 0, 70);
    const currentStatus = parseStatusBuffer(result);
    console.log(`✔ 耗时: ${Date.now() - startDate.getTime()}ms`);
    // console.log(JSON.stringify(currentStatus));

    // 任务切换时，进入卷帘门逻辑
    if (currentStatus.tr_step_index !== status?.tr_step_index) {
      // 检查上一个任务是否有卷帘门
      const lastDoor = DoorService.getInstance().getDoorFromCargoSize(
        status?.tr_odo_total
      );
      if (lastDoor) {
        // 1.取消监听卷帘门状态
        MqttClient.getInstance().unsubscribe(`/iot/fhjlm/pub/${lastDoor.ip}`);
        // 2.如果发送过开门指令，则需要发送mqtt指令: 5秒后关门
        if (status?.openCommandSended) {
          setTimeout(() => {
            sendMessageToDoor({
              clientId: client.id,
              doorIp: lastDoor.ip,
              command: "close",
            });
          }, 5000);
        }
      }

      // 检查当前任务是否有卷帘门
      const currentDoor = DoorService.getInstance().getDoorFromCargoSize(
        currentStatus.tr_odo_total
      );
      console.log(
        `当前分步: ${currentStatus.tr_step_index}, ${currentStatus.id_a} 到 ${currentStatus.id_b}`,
        currentDoor
          ? `, 该路段有门: ${currentDoor?.ip}, ${currentStatus.tr_odo_total}`
          : ""
      );
      if (currentDoor) {
        // 1.agv发送指令: 停车
        client.writeSingleCoil(0, true);
        // 2.监听卷帘门状态
        MqttClient.getInstance().subscribe({
          topic: `/iot/fhjlm/pub/${currentDoor.ip}`,
          callback: (_, msg) => {
            const data = JSON.parse(msg.toString());
            if (data["openstate"] === 1) {
              // agv发送指令: 解除停车
              client.writeSingleCoil(0, false);
            } else {
              // agv发送指令: 停车
              client.writeSingleCoil(0, true);
              // mqtt发送指令: 开门一次
              if (!status?.openCommandSended) {
                sendMessageToDoor({
                  clientId: client.id,
                  doorIp: currentDoor.ip,
                  command: "open",
                });
                currentStatus.openCommandSended = true;
              }
            }
          },
        });
      }
    } else {
      // 记录当前步骤，是否发送过开门指令
      currentStatus.openCommandSended = status?.openCommandSended;
    }

    // 记录agv最新状态
    agvStatus.set(client.id, currentStatus);

    // 写入错误日志
    if (
      currentStatus.tr_check_state == 3 ||
      currentStatus.tr_check_error > 0 ||
      currentStatus.err1 > 0 ||
      currentStatus.err2 > 0 ||
      currentStatus.err3 > 0 ||
      currentStatus.err4 > 0 ||
      currentStatus.err5 > 0 ||
      currentStatus.warnings > 0
    ) {
      insertAlarm({
        agv_id: client.id,
        tr_check_state: currentStatus.tr_check_state,
        tr_check_error: currentStatus.tr_check_error,
        err1: currentStatus.err1,
        err2: currentStatus.err2,
        err3: currentStatus.err3,
        err4: currentStatus.err4,
        err5: currentStatus.err5,
        warnings: currentStatus.warnings,
        code: result.toString("hex"),
        body: currentStatus,
      });
    }

    status = currentStatus; // 给状态赋值
    isOnline = true; // 标记为在线
  } catch (err: any) {
    console.error(`❌ modbus读取出错: ${err.message}`);
    client.destroy();
  }
  return {
    id: client.id,
    isOnline,
    trCpltPercent: status?.tr_cplt_percent,
    position: status ? { x: status.px, y: status.py } : null,
    battery: status?.energy_level,
    chargeMode: status?.status_charge,
    speed: status?.traction_speed,
    liftHeight: status?.lift_height,
    angle: status?.pa,
    timestamp: Date.now(),
  };
}

async function sendMessageToModbus(args: {
  ws: WebSocket;
  client: ModbusClient;
  type: string;
  steps?: SubTask[];
}) {
  const { ws, client, type, steps } = args;
  try {
    switch (type) {
      case "stop":
        await stopHandler(client);
        break;

      default:
        await schedulingHandler(client, type, steps ?? []);
        break;
    }
  } catch (error: any) {
    insertAlarm({
      agv_id: client.id,
      tr_check_state: 0,
      tr_check_error: 1000, // 检查错误
      err1: 0,
      err2: 0,
      err3: 0,
      err4: 0,
      err5: 0,
      warnings: 0,
      code: "",
      body: { type, steps, message: error.message },
    });
    WebSocketClient.getInstance().sendData(ws, {
      type: "error",
      data: {
        title: "任务发送异常",
        message: error.message ?? "未知错误",
      },
    });
  }
}

export async function schedulingHandler(
  client: ModbusClient,
  type: string,
  steps: SubTask[]
) {
  const st = Date.now();
  // 传输任务
  const cmds = covertToTaskBuffer(TaskType.send, steps);

  // 分批次写入
  const MAX_BYTES_PER_BATCH = 200;
  for (let i = 0; i < cmds.length; i += MAX_BYTES_PER_BATCH) {
    const registerAddress = Math.floor(i / 2);
    const batch = cmds.subarray(i, i + MAX_BYTES_PER_BATCH);
    await client.writeSingleRegisters(registerAddress, batch);
  }

  // 执行任务
  const executeCmd = covertToTaskBuffer(TaskType.execute);
  await client.writeSingleRegisters(0, executeCmd);
  await client.writeSingleCoil(0, false);
  setTimeout(async () => {
    await client.writeSingleRegisters(0, executeCmd);
    await client.writeSingleRegisters(0, executeCmd);

    // 解除原地等待
    const activeCmd = covertToTaskBuffer(TaskType.active);
    await client.writeSingleRegisters(0, activeCmd);

    // 写入日志
    const cmdsStr = cmds.toString("hex");
    insertTask({
      agv_id: client.id,
      type,
      duration: Date.now() - st,
      code: cmdsStr,
      body: parseTaskHexString(cmdsStr),
    });
  }, 1000);
}

export async function stopHandler(client: ModbusClient) {
  const st = Date.now();
  // 取消订阅所有门
  for (const door of DoorService.getInstance().getDoors()) {
    MqttClient.getInstance().unsubscribe(`/iot/fhjlm/pub/${door.ip}`);
  }
  // 暂停任务
  const pauseCmd = covertToTaskBuffer(TaskType.pause);
  await client.writeSingleRegisters(0, pauseCmd);
  setTimeout(() => {
    // 停止任务
    const stopCmd = covertToTaskBuffer(TaskType.stop);
    client.writeSingleRegisters(0, stopCmd);

    // 写入日志
    insertTask({
      agv_id: client.id,
      type: "stop",
      duration: Date.now() - st,
      code: null,
      body: null,
    });
  }, 300);
}

function sendMessageToDoor(args: {
  clientId: string;
  doorIp: string;
  command: "open" | "close";
}) {
  const { clientId, doorIp, command } = args;
  const body = {
    topic: `/iot/fhjlm/sub/${doorIp}`,
    message: JSON.stringify({
      w: [{ tag: "opencommand", value: command === "open" ? 1 : 0 }],
    }),
  };
  // 1.发送mqtt指令
  MqttClient.getInstance().publish(body);
  // 2.广播开门消息给客户端
  WebSocketClient.getInstance().broadcastData({
    type: "info",
    data: {
      title: `发送${command === "open" ? "开门" : "关门"}指令`,
      message: `卷帘门IP地址: ${doorIp}`,
    },
  });
  // 3.写入任务日志
  insertTask({
    agv_id: clientId,
    type: `${command}_door`,
    duration: 1,
    code: null,
    body,
  });
}
