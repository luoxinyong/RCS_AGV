import { Point, ArcData } from "./types";

// 点到线段上最近点的参数
export const getClosestPointOnLine = (
  point: { x: number; y: number },
  lineStart: Point,
  lineEnd: Point
): number => {
  const A = point.x - lineStart.x;
  const B = point.y - lineStart.y;
  const C = lineEnd.x - lineStart.x;
  const D = lineEnd.y - lineStart.y;

  const dot = A * C + B * D;
  const lenSq = C * C + D * D;

  return lenSq !== 0 ? dot / lenSq : -1;
};

// 三点定弧算法
export const createArcFromThreePoints = (
  p1: Point,
  p2: Point,
  p3: Point
): ArcData | null => {
  const ax = p1.x,
    ay = p1.y;
  const bx = p2.x,
    by = p2.y;
  const cx = p3.x,
    cy = p3.y;

  const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

  if (Math.abs(d) < 1e-6) return null; // 三点共线

  const ux =
    ((ax * ax + ay * ay) * (by - cy) +
      (bx * bx + by * by) * (cy - ay) +
      (cx * cx + cy * cy) * (ay - by)) /
    d;
  const uy =
    ((ax * ax + ay * ay) * (cx - bx) +
      (bx * bx + by * by) * (ax - cx) +
      (cx * cx + cy * cy) * (bx - ax)) /
    d;

  const radius = Math.sqrt((ux - ax) * (ux - ax) + (uy - ay) * (uy - ay));

  return {
    centerX: ux,
    centerY: uy,
    radius: radius,
    startAngle: Math.atan2(ay - uy, ax - ux),
    middleAngle: Math.atan2(by - uy, bx - ux),
    endAngle: Math.atan2(cy - uy, cx - ux),
  };
};

// 屏幕坐标转世界坐标
export const screenToWorld = (
  screen: { x: number; y: number },
  offset: { x: number; y: number },
  rect: DOMRect,
  scale: number
) => {
  return {
    x: (screen.x - rect.left - offset.x) / scale,
    y: (screen.y - rect.top - offset.y) / scale,
  };
};

// 世界坐标转屏幕坐标
export const worldToScreen = (
  world: { x: number; y: number },
  offset: { x: number; y: number },
  scale: number
) => {
  return {
    x: world.x * scale + offset.x,
    y: world.y * scale + offset.y,
  };
};
