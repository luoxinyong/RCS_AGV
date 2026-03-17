// 点的数据结构
export interface Point {
  id: number;
  x: number;
  y: number;
  selected?: boolean; // 是否被选中
  hovered?: boolean; // 是否被悬浮
}

// 线的数据结构
export interface Line {
  id: number;
  startPoint: Point;
  endPoint: Point;
  type: "straight" | "arc";
  isVertical?: boolean;
  isHorizontal?: boolean;
  fixedLength?: number;
  selected?: boolean; // 是否被选中
  hovered?: boolean; // 是否被悬浮
  // 弧线专用
  middlePoint?: Point;
}

// 绘制模式
export type DrawMode = "none" | "straight" | "arc" | "select" | "boxSelect";

// 约束模式 - 改为使用布尔值分别控制
export interface ConstraintOptions {
  isOrthogonal: boolean;
  isFixedLength: boolean;
  fixedLength: number;
}

// 单位和刻度
export interface UnitInfo {
  unit: string;
  factor: number;
  step: number;
}

// 弧形数据结构
export interface ArcData {
  centerX: number;
  centerY: number;
  radius: number;
  startAngle: number;
  middleAngle: number;
  endAngle: number;
}
