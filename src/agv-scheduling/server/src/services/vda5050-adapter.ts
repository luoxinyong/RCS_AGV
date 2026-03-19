/**
 * VDA5050 v2.0 适配器
 *
 * 职责：在现有 Modbus 调度层上方提供 VDA5050 标准 MQTT 接口
 * - 订阅 Order / InstantActions → 翻译为 SubTask[] → 调用 schedulingHandler 下发
 * - 读取 agvStatus → 翻译为 VDA5050 State → 定时发布到 MQTT
 * - 管理 Connection 状态（retained 消息）
 *
 * MQTT 主题格式（VDA5050 v2.0）：
 *   uagv/v2/{manufacturer}/{serialNumber}/order          ← 接收
 *   uagv/v2/{manufacturer}/{serialNumber}/instantActions  ← 接收
 *   uagv/v2/{manufacturer}/{serialNumber}/state           → 发布
 *   uagv/v2/{manufacturer}/{serialNumber}/connection      → 发布（retained）
 */

import MqttClient from "./mqtt";
import APP_CONFIG from "../config/app";
import { agvStatus, modbusClients, schedulingHandler, stopHandler } from "./scheduler";
import type { SubTask } from "../utils/modbus";
import type { AGVStatus } from "../types/device";
import type {
  VDA5050Order,
  VDA5050InstantActions,
  VDA5050State,
  VDA5050Connection,
  VDA5050Node,
  VDA5050Edge,
  AGVOrderState,
  ActionState,
  NodeState,
  EdgeState,
  VDA5050Error,
  Action,
} from "../types/vda5050";

// ========== 坐标工具函数 ==========

/** 将32位整数拆为两个16位整数（高位+低位），用于 Modbus 寄存器写入 */
function splitInt32ToInt16s(value: number): [number, number] {
  const low = value & 0xffff;
  const high = (value >> 16) & 0xffff;
  return [low, high];
}

/** 米 → 毫米 */
function metersToMm(m: number): number {
  return Math.round(m * 1000);
}

/** 毫米 → 米 */
function mmToMeters(mm: number): number {
  return mm / 1000;
}

/** 弧度 → 0.1度（内部角度单位） */
function radToTenthDeg(rad: number): number {
  return Math.round((rad * 180) / Math.PI * 10);
}

/** 0.1度 → 弧度 */
function tenthDegToRad(tenthDeg: number): number {
  return (tenthDeg / 10) * (Math.PI / 180);
}

/** 度 → 弧度 */
function degToRad(deg: number): number {
  return deg * (Math.PI / 180);
}

// ========== 适配器 ==========

export default class VDA5050Adapter {
  private static instance: VDA5050Adapter | null = null;
  private running = false;
  private stateTimer: ReturnType<typeof setInterval> | null = null;
  private headerCounter = 0;
  private manufacturer: string;

  /** 每台 AGV 的订单跟踪状态 */
  private orderStates = new Map<string, AGVOrderState>();

  public static getInstance(): VDA5050Adapter {
    if (!VDA5050Adapter.instance) {
      VDA5050Adapter.instance = new VDA5050Adapter();
    }
    return VDA5050Adapter.instance;
  }

  private constructor() {
    this.manufacturer = APP_CONFIG.VDA5050_MANUFACTURER;
  }

  /** 注册所有 AGV 的 MQTT 订阅 */
  private subscribeAll(): void {
    const mqtt = MqttClient.getInstance();
    for (const client of modbusClients) {
      const agvId = client.id;
      const orderTopic = this.buildTopic(agvId, "order");
      const actionsTopic = this.buildTopic(agvId, "instantActions");

      mqtt.subscribe({
        topic: orderTopic,
        options: { qos: 1 },
        callback: (topic, message) => {
          if (topic !== orderTopic) return;
          this.handleOrder(agvId, message);
        },
      });

      mqtt.subscribe({
        topic: actionsTopic,
        options: { qos: 1 },
        callback: (topic, message) => {
          if (topic !== actionsTopic) return;
          this.handleInstantActions(agvId, message);
        },
      });

      this.publishConnection(agvId, "ONLINE");
    }
  }

  /** 启动适配器：订阅 MQTT 主题 + 启动状态发布定时器 */
  public start(): void {
    if (this.running) return;

    const mqtt = MqttClient.getInstance();
    if (mqtt.isConnected()) {
      // MQTT 已连接，直接订阅
      this.subscribeAll();
    } else {
      // MQTT 未连接，等 MQTT configure 的 onConnected 或 resubscribeAll 自动处理
      console.log("⚠️ VDA5050: MQTT 未连接，将在连接建立后自动订阅");
    }

    // 每秒发布 State（MQTT 未连接时 publish 内部会静默跳过）
    this.stateTimer = setInterval(() => {
      if (mqtt.isConnected()) this.publishAllStates();
    }, 1000);

    this.running = true;
    console.log(`✅ VDA5050 适配器已启动，制造商: ${this.manufacturer}，AGV数量: ${modbusClients.length}`);
  }

  /** MQTT 连接/重连后调用，注册订阅 */
  public onMqttConnected(): void {
    if (!this.running) return;
    console.log("📡 VDA5050: MQTT 已连接，注册订阅...");
    this.subscribeAll();
  }

  /** 停止适配器 */
  public stop(): void {
    if (this.stateTimer) {
      clearInterval(this.stateTimer);
      this.stateTimer = null;
    }
    // 发布 OFFLINE 状态
    for (const client of modbusClients) {
      this.publishConnection(client.id, "OFFLINE");
    }
    this.running = false;
    console.log("🛑 VDA5050 适配器已停止");
  }

  // ========== MQTT 主题构建 ==========

  private buildTopic(agvId: string, suffix: string): string {
    return `uagv/v2/${this.manufacturer}/${agvId}/${suffix}`;
  }

  private nextHeaderId(): number {
    return ++this.headerCounter;
  }

  private isoTimestamp(): string {
    return new Date().toISOString();
  }

  // ========== Order 处理 ==========

  private handleOrder(agvId: string, message: Buffer): void {
    try {
      const order: VDA5050Order = JSON.parse(message.toString());
      console.log(`📥 VDA5050 收到 Order: agv=${agvId}, orderId=${order.orderId}, updateId=${order.orderUpdateId}`);

      // 翻译 Order → SubTask[]
      const steps = this.translateOrderToSubTasks(order);
      if (steps.length === 0) {
        console.warn(`⚠️ VDA5050: Order ${order.orderId} 翻译后无有效步骤`);
        return;
      }

      // 初始化订单跟踪状态
      this.initOrderState(agvId, order, steps.length);

      // 查找对应的 Modbus 客户端并下发
      const client = modbusClients.find((c) => c.id === agvId);
      if (!client) {
        console.error(`❌ VDA5050: 未找到 AGV ${agvId} 对应的 Modbus 客户端`);
        return;
      }

      schedulingHandler(client, "vda5050", steps).then(() => {
        console.log(`✅ VDA5050: Order ${order.orderId} 已下发到 AGV ${agvId}，共 ${steps.length} 步`);
      }).catch((err) => {
        console.error(`❌ VDA5050: Order 下发失败:`, err.message);
      });
    } catch (err: any) {
      console.error(`❌ VDA5050: 解析 Order 失败:`, err.message);
    }
  }

  // ========== InstantActions 处理 ==========

  private handleInstantActions(agvId: string, message: Buffer): void {
    try {
      const msg: VDA5050InstantActions = JSON.parse(message.toString());
      console.log(`📥 VDA5050 收到 InstantActions: agv=${agvId}, actions=${msg.actions.length}`);

      const client = modbusClients.find((c) => c.id === agvId);
      if (!client) {
        console.error(`❌ VDA5050: 未找到 AGV ${agvId}`);
        return;
      }

      for (const action of msg.actions) {
        this.executeInstantAction(agvId, client, action);
      }
    } catch (err: any) {
      console.error(`❌ VDA5050: 解析 InstantActions 失败:`, err.message);
    }
  }

  private async executeInstantAction(agvId: string, client: any, action: Action): Promise<void> {
    console.log(`⚡ VDA5050 InstantAction: ${action.actionType} on AGV ${agvId}`);
    try {
      switch (action.actionType) {
        case "cancelOrder":
          await stopHandler(client);
          this.orderStates.delete(agvId);
          break;

        case "startPause":
          // 写入停车 coil
          await client.writeSingleCoil(0, true);
          break;

        case "stopPause":
          // 解除停车
          await client.writeSingleCoil(0, false);
          break;

        default:
          console.warn(`⚠️ VDA5050: 不支持的 InstantAction: ${action.actionType}`);
      }
    } catch (err: any) {
      console.error(`❌ VDA5050: InstantAction ${action.actionType} 执行失败:`, err.message);
    }
  }

  // ========== Order → SubTask[] 翻译 ==========

  private translateOrderToSubTasks(order: VDA5050Order): SubTask[] {
    const steps: SubTask[] = [];
    const releasedNodes = order.nodes.filter((n) => n.released);
    const releasedEdges = order.edges.filter((e) => e.released);

    // 按 sequenceId 排序
    releasedNodes.sort((a, b) => a.sequenceId - b.sequenceId);
    releasedEdges.sort((a, b) => a.sequenceId - b.sequenceId);

    // 遍历 Node-Edge 序列
    for (let i = 0; i < releasedNodes.length; i++) {
      const node = releasedNodes[i];

      // 处理 Node 上的 Actions（到达节点时执行）
      for (const action of node.actions) {
        const actionStep = this.translateAction(action, node);
        if (actionStep) steps.push(actionStep);
      }

      // 处理从当前节点到下一个节点的 Edge（移动）
      if (i < releasedNodes.length - 1) {
        const nextNode = releasedNodes[i + 1];
        // 找到对应的 edge
        const edge = releasedEdges.find(
          (e) => e.startNodeId === node.nodeId && e.endNodeId === nextNode.nodeId
        );

        if (edge && node.nodePosition && nextNode.nodePosition) {
          const moveStep = this.translateEdge(edge, node, nextNode);
          steps.push(moveStep);

          // 处理 Edge 上的 Actions
          for (const action of edge.actions) {
            const actionStep = this.translateAction(action, nextNode);
            if (actionStep) steps.push(actionStep);
          }
        }
      }
    }

    // 添加任务结束标记
    if (steps.length > 0) {
      steps.push({ offset1: 99 });
    }

    return steps;
  }

  /** 翻译 Edge（两节点间移动）→ SubTask（直线行驶） */
  private translateEdge(edge: VDA5050Edge, fromNode: VDA5050Node, toNode: VDA5050Node): SubTask {
    const fromPos = fromNode.nodePosition!;
    const toPos = toNode.nodePosition!;

    // 坐标：VDA5050 米 → 内部毫米
    const ax = metersToMm(fromPos.x);
    const ay = metersToMm(fromPos.y);
    const bx = metersToMm(toPos.x);
    const by = metersToMm(toPos.y);

    // 计算行驶距离（毫米）
    const dx = bx - ax;
    const dy = by - ay;
    const distance = Math.round(Math.sqrt(dx * dx + dy * dy));

    // 计算行驶方向（0.1度）
    const headingRad = Math.atan2(dy, dx);
    const heading = Math.round((headingRad * 180) / Math.PI * 10);

    // 速度限制：VDA5050 m/s → 内部 mm/s，默认 500 mm/s
    const speed = edge.maxSpeed ? Math.round(edge.maxSpeed * 1000) : 500;

    // 拆分坐标为两个 16 位整数
    const [axLow, axHigh] = splitInt32ToInt16s(ax);
    const [ayLow, ayHigh] = splitInt32ToInt16s(ay);
    const [bxLow, bxHigh] = splitInt32ToInt16s(bx);
    const [byLow, byHigh] = splitInt32ToInt16s(by);

    return {
      offset1: 1, // 直线行驶
      offset2: speed,
      offset3: heading,
      offset4: distance,
      offset5: 0, // cargo_size
      offset6: axLow,
      offset7: axHigh,
      offset8: ayLow,
      offset9: ayHigh,
      offset10: bxLow,
      offset11: bxHigh,
      offset12: byLow,
      offset13: byHigh,
    };
  }

  /** 翻译 Action → SubTask */
  private translateAction(action: Action, atNode: VDA5050Node): SubTask | null {
    const pos = atNode.nodePosition;
    const x = pos ? metersToMm(pos.x) : 0;
    const y = pos ? metersToMm(pos.y) : 0;

    switch (action.actionType) {
      case "pick": {
        // 取货：先自调节（offset1=9），再举升（offset1=3）
        // MVP 简化：只生成取货步骤
        const [xLow, xHigh] = splitInt32ToInt16s(x);
        const [yLow, yHigh] = splitInt32ToInt16s(y);
        return {
          offset1: 9, // 自调节取货
          offset2: 300, // 默认速度
          offset3: pos?.theta ? radToTenthDeg(pos.theta) : 0,
          offset4: 0, // 距离
          offset5: 0,
          offset6: xLow,
          offset7: xHigh,
          offset8: yLow,
          offset9: yHigh,
        };
      }

      case "drop": {
        // 放货：举升（offset1=3）
        const height = this.getActionParam(action, "height", 0) as number;
        const [xLow, xHigh] = splitInt32ToInt16s(x);
        const [yLow, yHigh] = splitInt32ToInt16s(y);
        return {
          offset1: 3, // 举升
          offset2: height, // 举升高度
          offset3: 50, // 举升速度
          offset8: xLow,
          offset9: xHigh,
          offset10: yLow,
          offset11: yHigh,
        };
      }

      case "charge": {
        // 充电：offset1=4
        const [xLow, xHigh] = splitInt32ToInt16s(x);
        const [yLow, yHigh] = splitInt32ToInt16s(y);
        return {
          offset1: 4, // 充电
          offset2: 1, // 充电模式
          offset3: 100, // 充电上限
          offset5: pos?.theta ? radToTenthDeg(pos.theta) : 0,
          offset6: xLow,
          offset7: xHigh,
          offset8: yLow,
          offset9: yHigh,
        };
      }

      default:
        // 不支持的 action 类型跳过
        console.log(`⚠️ VDA5050: 跳过不支持的 Action: ${action.actionType}`);
        return null;
    }
  }

  /** 从 Action 参数中获取值 */
  private getActionParam(action: Action, key: string, defaultValue: string | number | boolean): string | number | boolean {
    const param = action.actionParameters?.find((p) => p.key === key);
    return param ? param.value : defaultValue;
  }

  // ========== 订单状态跟踪 ==========

  private initOrderState(agvId: string, order: VDA5050Order, totalSteps: number): void {
    const releasedNodes = order.nodes.filter((n) => n.released);
    const releasedEdges = order.edges.filter((e) => e.released);

    // 初始 nodeStates：所有节点都待经过
    const nodeStates: NodeState[] = releasedNodes.map((n) => ({
      nodeId: n.nodeId,
      sequenceId: n.sequenceId,
      released: n.released,
      nodePosition: n.nodePosition,
    }));

    // 初始 edgeStates：所有边都待经过
    const edgeStates: EdgeState[] = releasedEdges.map((e) => ({
      edgeId: e.edgeId,
      sequenceId: e.sequenceId,
      released: e.released,
    }));

    // 收集所有 actions 的状态
    const actionStates: ActionState[] = [];
    for (const node of releasedNodes) {
      for (const action of node.actions) {
        actionStates.push({
          actionId: action.actionId,
          actionType: action.actionType,
          actionStatus: "WAITING",
        });
      }
    }
    for (const edge of releasedEdges) {
      for (const action of edge.actions) {
        actionStates.push({
          actionId: action.actionId,
          actionType: action.actionType,
          actionStatus: "WAITING",
        });
      }
    }

    this.orderStates.set(agvId, {
      orderId: order.orderId,
      orderUpdateId: order.orderUpdateId,
      nodes: releasedNodes,
      edges: releasedEdges,
      nodeStates,
      edgeStates,
      actionStates,
      lastNodeId: releasedNodes[0]?.nodeId || "",
      lastNodeSequenceId: releasedNodes[0]?.sequenceId || 0,
      totalSteps,
    });
  }

  /** 根据 AGV 当前步骤进度更新订单状态 */
  private updateOrderProgress(agvId: string, status: AGVStatus): void {
    const orderState = this.orderStates.get(agvId);
    if (!orderState) return;

    const { tr_step_index, tr_cplt_percent } = status;

    // 粗略映射：根据步骤索引推算已经过的节点/边
    // 每个 node-edge 对大约对应 1~3 个 SubTask 步骤
    const progressRatio = orderState.totalSteps > 0
      ? tr_step_index / orderState.totalSteps
      : 0;

    // 更新已经过的节点
    const passedNodeCount = Math.min(
      Math.floor(progressRatio * orderState.nodes.length) + 1,
      orderState.nodes.length
    );

    // 移除已经过的节点/边（VDA5050 state 只报告剩余的）
    if (passedNodeCount > 1) {
      const lastPassed = orderState.nodes[passedNodeCount - 1];
      if (lastPassed) {
        orderState.lastNodeId = lastPassed.nodeId;
        orderState.lastNodeSequenceId = lastPassed.sequenceId;
      }
      // 保留未经过的
      orderState.nodeStates = orderState.nodeStates.slice(passedNodeCount - 1);
      orderState.edgeStates = orderState.edgeStates.slice(Math.max(0, passedNodeCount - 1));
    }

    // 任务完成时标记所有 action 为 FINISHED
    if (tr_cplt_percent >= 100) {
      for (const actionState of orderState.actionStates) {
        actionState.actionStatus = "FINISHED";
      }
    }
  }

  // ========== State 发布 ==========

  private publishAllStates(): void {
    for (const client of modbusClients) {
      const status = agvStatus.get(client.id);
      if (!status) continue;

      // 更新订单进度
      this.updateOrderProgress(client.id, status);

      // 构建并发布 State
      const state = this.buildState(client.id, status);
      MqttClient.getInstance().publish({
        topic: this.buildTopic(client.id, "state"),
        message: JSON.stringify(state),
        options: { qos: 0, retain: false },
      });
    }
  }

  private buildState(agvId: string, status: AGVStatus): VDA5050State {
    const orderState = this.orderStates.get(agvId);

    // 错误列表
    const errors: VDA5050Error[] = [];
    if (status.err1 > 0) errors.push({ errorType: `ERR1_${status.err1}`, errorLevel: "FATAL" });
    if (status.err2 > 0) errors.push({ errorType: `ERR2_${status.err2}`, errorLevel: "FATAL" });
    if (status.err3 > 0) errors.push({ errorType: `ERR3_${status.err3}`, errorLevel: "FATAL" });
    if (status.err4 > 0) errors.push({ errorType: `ERR4_${status.err4}`, errorLevel: "FATAL" });
    if (status.err5 > 0) errors.push({ errorType: `ERR5_${status.err5}`, errorLevel: "FATAL" });
    if (status.warnings > 0) errors.push({ errorType: `WARN_${status.warnings}`, errorLevel: "WARNING" });

    // 判断是否在行驶
    const driving = status.traction_is_parked === 0 && Math.abs(status.traction_speed) > 0;

    return {
      headerId: this.nextHeaderId(),
      timestamp: this.isoTimestamp(),
      version: "2.0.0",
      manufacturer: this.manufacturer,
      serialNumber: agvId,
      orderId: orderState?.orderId || "",
      orderUpdateId: orderState?.orderUpdateId || 0,
      lastNodeId: orderState?.lastNodeId || "",
      lastNodeSequenceId: orderState?.lastNodeSequenceId || 0,
      driving,
      paused: status.step_en_pause === 1,
      newBaseRequest: false,
      operatingMode: "AUTOMATIC",
      nodeStates: orderState?.nodeStates || [],
      edgeStates: orderState?.edgeStates || [],
      actionStates: orderState?.actionStates || [],
      batteryState: {
        batteryCharge: status.energy_level,
        charging: status.status_charge === 1,
      },
      errors,
      agvPosition: {
        x: mmToMeters(status.px),
        y: mmToMeters(status.py),
        theta: degToRad(status.pa), // pa 已经是度（parseStatusBuffer 除了10）
        mapId: "1",
        positionInitialized: true,
      },
      velocity: {
        vx: mmToMeters(status.traction_speed),
      },
      loads: status.cargo_loaded
        ? [{ loadId: "1", loadType: "pallet" }]
        : [],
      safetyState: {
        eStop: "NONE",
        fieldViolation: false,
      },
    };
  }

  // ========== Connection 发布 ==========

  private publishConnection(agvId: string, state: "ONLINE" | "OFFLINE" | "CONNECTIONBROKEN"): void {
    const connection: VDA5050Connection = {
      headerId: this.nextHeaderId(),
      timestamp: this.isoTimestamp(),
      version: "2.0.0",
      manufacturer: this.manufacturer,
      serialNumber: agvId,
      connectionState: state,
    };
    MqttClient.getInstance().publish({
      topic: this.buildTopic(agvId, "connection"),
      message: JSON.stringify(connection),
      options: { qos: 1, retain: true }, // Connection 消息用 retained
    });
  }
}
