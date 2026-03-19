/**
 * VDA5050 v2.0 类型定义（MVP范围）
 *
 * 参考标准：VDA 5050 Version 2.0.0
 * 仅定义适配层需要的核心接口，非完整协议实现
 */

// ========== 基础类型 ==========

export interface NodePosition {
  x: number; // 米
  y: number; // 米
  theta?: number; // 弧度，可选
  allowedDeviationXY?: number;
  allowedDeviationTheta?: number;
  mapId: string;
}

export interface Action {
  actionType: string; // "pick", "drop", "charge", "cancelOrder", "startPause", "stopPause"
  actionId: string;
  blockingType: "NONE" | "SOFT" | "HARD";
  actionDescription?: string;
  actionParameters?: ActionParameter[];
}

export interface ActionParameter {
  key: string;
  value: string | number | boolean;
}

// ========== Order 相关 ==========

export interface VDA5050Node {
  nodeId: string;
  sequenceId: number;
  released: boolean;
  nodePosition?: NodePosition;
  actions: Action[];
}

export interface VDA5050Edge {
  edgeId: string;
  sequenceId: number;
  released: boolean;
  startNodeId: string;
  endNodeId: string;
  maxSpeed?: number; // m/s
  maxHeight?: number;
  minHeight?: number;
  orientation?: number; // 弧度
  direction?: string;
  rotationAllowed?: boolean;
  length?: number; // 米
  actions: Action[];
}

export interface VDA5050Order {
  headerId: number;
  timestamp: string; // ISO 8601
  version: string;
  manufacturer: string;
  serialNumber: string;
  orderId: string;
  orderUpdateId: number;
  zoneSetId?: string;
  nodes: VDA5050Node[];
  edges: VDA5050Edge[];
}

// ========== InstantActions ==========

export interface VDA5050InstantActions {
  headerId: number;
  timestamp: string;
  version: string;
  manufacturer: string;
  serialNumber: string;
  actions: Action[];
}

// ========== State 相关 ==========

export interface AGVPosition {
  x: number; // 米
  y: number; // 米
  theta: number; // 弧度
  mapId: string;
  positionInitialized: boolean;
  deviationRange?: number;
}

export interface Velocity {
  vx?: number; // m/s
  vy?: number; // m/s
  omega?: number; // rad/s
}

export interface BatteryState {
  batteryCharge: number; // 0~100
  batteryVoltage?: number;
  batteryHealth?: number;
  charging: boolean;
  reach?: number;
}

export interface Load {
  loadId?: string;
  loadType?: string;
  loadPosition?: string;
  boundingBoxReference?: { x: number; y: number; z: number };
  loadDimensions?: { length: number; width: number; height: number };
  weight?: number;
}

export type ActionStatus = "WAITING" | "INITIALIZING" | "RUNNING" | "PAUSED" | "FINISHED" | "FAILED";

export interface ActionState {
  actionId: string;
  actionType: string;
  actionStatus: ActionStatus;
  actionDescription?: string;
  resultDescription?: string;
}

export interface NodeState {
  nodeId: string;
  sequenceId: number;
  nodeDescription?: string;
  nodePosition?: NodePosition;
  released: boolean;
}

export interface EdgeState {
  edgeId: string;
  sequenceId: number;
  edgeDescription?: string;
  released: boolean;
}

export type ErrorLevel = "WARNING" | "FATAL";

export interface VDA5050Error {
  errorType: string;
  errorLevel: ErrorLevel;
  errorDescription?: string;
  errorReferences?: Array<{ referenceKey: string; referenceValue: string }>;
}

export type OperatingMode = "AUTOMATIC" | "SEMIAUTOMATIC" | "MANUAL" | "SERVICE" | "TEACHIN";

export interface SafetyState {
  eStop: "AUTOACK" | "MANUAL" | "REMOTE" | "NONE";
  fieldViolation: boolean;
}

export interface VDA5050State {
  headerId: number;
  timestamp: string;
  version: string;
  manufacturer: string;
  serialNumber: string;
  orderId: string;
  orderUpdateId: number;
  zoneSetId?: string;
  lastNodeId: string;
  lastNodeSequenceId: number;
  driving: boolean;
  paused: boolean;
  newBaseRequest: boolean;
  distanceSinceLastNode?: number;
  operatingMode: OperatingMode;
  nodeStates: NodeState[];
  edgeStates: EdgeState[];
  actionStates: ActionState[];
  batteryState: BatteryState;
  errors: VDA5050Error[];
  agvPosition?: AGVPosition;
  velocity?: Velocity;
  loads?: Load[];
  safetyState: SafetyState;
}

// ========== Connection ==========

export type ConnectionState = "ONLINE" | "OFFLINE" | "CONNECTIONBROKEN";

export interface VDA5050Connection {
  headerId: number;
  timestamp: string;
  version: string;
  manufacturer: string;
  serialNumber: string;
  connectionState: ConnectionState;
}

// ========== 适配器内部用 ==========

/** 每台 AGV 的订单跟踪状态 */
export interface AGVOrderState {
  orderId: string;
  orderUpdateId: number;
  /** 原始 Order 的 nodes */
  nodes: VDA5050Node[];
  /** 原始 Order 的 edges */
  edges: VDA5050Edge[];
  /** 当前节点状态（待经过的） */
  nodeStates: NodeState[];
  /** 当前边状态（待经过的） */
  edgeStates: EdgeState[];
  /** 动作状态 */
  actionStates: ActionState[];
  /** 最后经过的节点 */
  lastNodeId: string;
  lastNodeSequenceId: number;
  /** 下发的 SubTask 步骤数 */
  totalSteps: number;
}
