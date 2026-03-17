/** 绘制模式 */
export type DrawMode = "none" | "point" | "select" | "straight" | "arc";

/** 任务运行模式 */
export type TaskMode = "pickDrop" | "move" | "charge" | "raise";

/** 弧线路径 */
export type ArcRoute = LineRoute & { middle: Point; center: Point };

/** 直线路径 */
export interface LineRoute {
  /** 起点 */
  start: Point;
  /** 终点 */
  end: Point;
  /** 起终点距离 */
  distance: number;
  /** 路径配置: 正方向: startId-endId，反方向: endId-startId */
  settings?: { [key: string]: RouteSettings };
}

/** 路径设置 */
export interface RouteSettings {
  /** 是否绕障 */
  isObstacleDetour: boolean;
  /** 避障区域 */
  obstacleAreas: number[];
  /** 限速 */
  limitSpeed?: number;
  /** 卷帘门 */
  doorId?: number;
}

/** 坐标点 */
export interface Point {
  /** 坐标点id，预览点没有值 */
  id?: string;
  /** 是否取放点，默认false */
  isPickOrDrop?: boolean;
  /** x轴 */
  x: number;
  /** y轴 */
  y: number;
  /** 坐标点配置，例如: 关闭避障区域(自旋时) */
  settings?: PointSettings;
}

/** 坐标点设置 */
export interface PointSettings {
  /** 避障区域（自旋时） */
  obstacleAreas: number[];
  /** 自旋方向 */
  rotationDirection?: number;
}

/** 直线的约束条件 */
export interface ConstraintOptions {
  /** 是否正交 */
  isOrthogonal: boolean;
  /** 是否固定长度 */
  isFixedLength: boolean;
  /** 固定长度值 */
  fixedLength: number;
}

/** AGV的实时状态 */
export interface AGVStatus {
  /** AGV编号 */
  id: string;
  /** 状态 */
  isOnline: boolean;
  /** 任务完成百分比，0~100 */
  trCpltPercent: number;
  /** 充电状态：0.未知 1.充电中 2.停止充电 */
  chargeMode: number;
  /** 位置 */
  position: { x: number; y: number };
  /** 电池电量 */
  battery: number;
  /** 速度 */
  speed: number;
  /** 角度 */
  angle: number;
  /** 货叉高度 */
  liftHeight: number;
  /** 时间戳 */
  timestamp: number;
}

/** 分布任务 */
export interface SubTask {
  offset1?: number;
  offset2?: number;
  offset3?: number;
  offset4?: number;
  offset5?: number;
  offset6?: number;
  offset7?: number;
  offset8?: number;
  offset9?: number;
  offset10?: number;
  offset11?: number;
  offset12?: number;
  offset13?: number;
  offset14?: number;
  offset15?: number;
  offset16?: number;
  offset17?: number;
  offset18?: number;
  offset19?: number;
  offset20?: number;
}
