import { fabric } from "fabric";
import { calculateArcDistance, calculateLineDistance } from ".";
import type { Point, AGVStatus, LineRoute, ArcRoute } from "@/types/map-editor";

/** 创建fabric坐标点 */
export const createCircle = (args: { center: Point; isPreview?: boolean }) => {
  const { x, y, id, isPickOrDrop, settings } = args.center;
  const isCharge = settings?.isCharge === true;
  const radius = 4;
  const pointColor = isCharge ? "#faad14" : isPickOrDrop ? "#ff6b6b" : "#36cfc9";

  const result: fabric.Object[] = [
    // 圆点
    new fabric.Circle({
      name: id,
      left: x - radius,
      top: y - radius,
      radius: radius,
      fill: pointColor,
      evented: false,
      stroke: "#ddd",
      strokeWidth: 2,
      hasControls: false,
      hasRotatingPoint: false,
      lockScalingX: true,
      lockScalingY: true,
      lockRotation: true,
      lockSkewingX: true,
      lockSkewingY: true,
      lockMovementX: true,
      lockMovementY: true,
      lockScalingFlip: true,
      hoverCursor: "pointer",
      data: {
        center: args.center,
        isPreview: args.isPreview,
      },
    }),
  ];
  // 文本标签显示ID
  if (id) {
    result.push(
      new fabric.Text((isCharge ? "C" : isPickOrDrop ? "Q" : "P") + id, {
        name: id,
        fontSize: 10,
        left: x,
        top: y - 10,
        originX: "center",
        originY: "center",
        fill: "#333",
        evented: false,
        hasControls: false,
        hasRotatingPoint: false,
        lockScalingX: true,
        lockScalingY: true,
        lockRotation: true,
        lockSkewingX: true,
        lockSkewingY: true,
        lockMovementX: true,
        lockMovementY: true,
        lockScalingFlip: true,
        hoverCursor: "pointer",
      })
    );
  }
  return result;
};

/** 创建fabric直线 */
export const createLine = (args: {
  start: Point;
  end: Point;
  isPreview?: boolean;
}) => {
  const { start, end, isPreview = false } = args;
  const lineRoute: LineRoute = {
    start,
    end,
    distance: calculateLineDistance(start, end),
  };
  return new fabric.Line([start.x, start.y, end.x, end.y], {
    stroke: "#40a9ff",
    strokeWidth: 2,
    strokeDashArray: isPreview ? [5, 5] : undefined,
    strokeLineCap: "round",
    evented: false,
    hasControls: false,
    hasRotatingPoint: false,
    lockScalingX: true,
    lockScalingY: true,
    lockRotation: true,
    lockSkewingX: true,
    lockSkewingY: true,
    lockMovementX: true,
    lockMovementY: true,
    lockScalingFlip: true,
    hoverCursor: "pointer",
    data: { ...lineRoute, isPreview },
  });
};

/** 创建fabric弧线 */
export const createPath = (args: {
  start: Point;
  middle: Point;
  end: Point;
  isPreview?: boolean;
}) => {
  const { start, middle, end, isPreview = false } = args;

  // 辅助函数：根据三点计算圆的中心和半径
  function getCircleFromThreePoints(p1: Point, p2: Point, p3: Point) {
    // 计算三条边的中点
    const mid1 = { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
    const mid2 = { x: (p2.x + p3.x) / 2, y: (p2.y + p3.y) / 2 };

    // 计算两条边的垂直平分线
    const slope1 = (p2.y - p1.y) / (p2.x - p1.x);
    const slopePerp1 = -1 / slope1;

    const slope2 = (p3.y - p2.y) / (p3.x - p2.x);
    const slopePerp2 = -1 / slope2;

    // 计算两条垂直平分线的交点（圆心）
    const x =
      (mid2.y - mid1.y + slopePerp1 * mid1.x - slopePerp2 * mid2.x) /
      (slopePerp1 - slopePerp2);
    const y = mid1.y + slopePerp1 * (x - mid1.x);

    const center = { x, y };
    const radius = Math.sqrt((p1.x - x) ** 2 + (p1.y - y) ** 2);

    return { center, radius };
  }

  // 辅助函数：判断三点是否顺时针方向
  function isClockwise(sAng: number, mAng: number, eAng: number): boolean {
    // 规范化角度到[0, 2π]
    const normalize = (angle: number) =>
      angle < 0 ? angle + 2 * Math.PI : angle;

    const sa = normalize(sAng);
    const ma = normalize(mAng);
    const ea = normalize(eAng);

    // 检查中间角度是否在起点和终点之间
    if (sa < ea) {
      return ma < sa || ma > ea;
    } else {
      return ma < sa && ma > ea;
    }
  }

  // 计算三点确定的圆弧的中心点和半径
  const { center, radius } = getCircleFromThreePoints(start, middle, end);

  // 计算起点、中间点和终点的角度，确定圆弧的方向（顺时针或逆时针）
  const startAngle = Math.atan2(start.y - center.y, start.x - center.x);
  const middleAngle = Math.atan2(middle.y - center.y, middle.x - center.x);
  const endAngle = Math.atan2(end.y - center.y, end.x - center.x);
  const sweepFlag = isClockwise(startAngle, middleAngle, endAngle) ? 0 : 1;

  // 构建圆弧路径命令
  try {
    const arcRoute: ArcRoute = {
      start,
      end,
      middle,
      center,
      distance: calculateArcDistance(start, center, end, sweepFlag === 0),
    };
    return new fabric.Path(
      `M ${start.x} ${start.y} A ${radius} ${radius} 0 0 ${sweepFlag} ${end.x} ${end.y}`,
      {
        fill: "",
        stroke: "#40a9ff",
        strokeWidth: 2,
        strokeDashArray: isPreview ? [5, 5] : undefined,
        strokeLineCap: "round",
        evented: false,
        hasControls: false,
        hasRotatingPoint: false,
        lockScalingX: true,
        lockScalingY: true,
        lockRotation: true,
        lockSkewingX: true,
        lockSkewingY: true,
        lockMovementX: true,
        lockMovementY: true,
        lockScalingFlip: true,
        hoverCursor: "pointer",
        data: { ...arcRoute, isPreview },
      }
    );
  } catch {
    return null;
  }
};

/** 创建agv对象 */
export const createAgv = (args: {
  status: AGVStatus;
  angleFixed: number;
  positionFixed: { x: number; y: number };
}) => {
  const { status, angleFixed, positionFixed } = args;
  const radius = 6;

  // 创建AGV车身（矩形）
  const circle = new fabric.Circle({
    radius: radius,
    fill: status.isOnline ? "#FF5722" : "#bfbfbf",
    stroke: status.isOnline ? "orange" : "#d9d9d9",
    originX: "center",
    originY: "center",
  });

  // 创建AGV ID标签
  const label = new fabric.Text(status.id, {
    fontSize: 10,
    fill: "#FFF",
    originX: "center",
    originY: "center",
  });

  // 创建车头方向指示器（三角形）
  const angleRad = (angleFixed * Math.PI) / 180; // 转换为弧度
  const distance = radius + 1.5; // 三角形到圆心的距离
  const triangle = new fabric.Triangle({
    left: Math.sin(angleRad) * distance,
    top: -Math.cos(angleRad) * distance,
    width: radius,
    height: radius * 0.75,
    fill: status.isOnline ? "#FF5722" : "#bfbfbf",
    stroke: status.isOnline ? "orange" : "#d9d9d9",
    originX: "center",
    originY: "center",
    angle: angleFixed,
  });

  // 创建组合对象
  return new fabric.Group([triangle, circle, label], {
    left: positionFixed.x,
    top: positionFixed.y,
    originX: "center",
    originY: "center",
    selectable: false,
    evented: false,
    data: status,
  });
};
