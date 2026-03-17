import { agvKey } from "@/config";
import type { ArcRoute, LineRoute, Point, SubTask } from "@/types/map-editor";

/** 查询最小的未被使用的正整数坐标点 */
export const findNextId = (points: Point[]) => {
  const usedNumber = new Set<number>();
  for (const pt of points) {
    const n = parseInt(pt.id ?? "0");
    if (!Number.isNaN(n) && n > 0) usedNumber.add(n);
  }

  let id = 1;
  while (usedNumber.has(id)) id++;
  return id.toString();
};

/** 查询距离范围内最近的坐标点 */
export const findClosestPoint = (args: {
  points: Point[];
  position: { x: number; y: number };
  minDistance: number;
}) => {
  const { points, position, minDistance } = args;
  let distance = minDistance;
  let closestPoint: Point | undefined;
  for (const pt of points) {
    const dx = pt.x - position.x;
    const dy = pt.y - position.y;
    const dt = Math.sqrt(dx * dx + dy * dy);
    if (dt < distance) {
      distance = dt;
      closestPoint = pt;
    }
  }
  return { closestPoint, distance };
};

/** 根据约束，获取直线终点 */
export const applyStraightConstraints = (args: {
  start: { x: number; y: number };
  end: { x: number; y: number };
  isOrthogonal: boolean;
  fixedLength?: number;
}) => {
  const { isOrthogonal, fixedLength } = args;
  const start = { x: args.start.x, y: args.start.y };
  const end = { x: args.end.x, y: args.end.y };

  // 实现正交约束
  if (isOrthogonal) {
    const dx = Math.abs(end.x - start.x);
    const dy = Math.abs(end.y - start.y);

    if (dx > dy) {
      end.y = start.y; // 保持水平
    } else {
      end.x = start.x; // 保持垂直
    }
  }

  // 实现固定长度约束
  if (fixedLength) {
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len > 0) {
      const factor = fixedLength / len;
      end.x = start.x + dx * factor;
      end.y = start.y + dy * factor;
    }
  }

  return end;
};

/** 计算直线距离 */
export function calculateLineDistance(
  a: { x: number; y: number },
  b: { x: number; y: number }
) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  return Math.sqrt(dx * dx + dy * dy);
}

/** 计算圆弧距离 */
export function calculateArcDistance(
  start: { x: number; y: number },
  center: { x: number; y: number },
  end: { x: number; y: number },
  isClockwise: boolean
): number {
  // 计算半径（圆心到起点的距离）
  const radius = calculateLineDistance(start, center);

  // 计算圆心到起点和终点的向量
  const startVec = { x: start.x - center.x, y: start.y - center.y };
  const endVec = { x: end.x - center.x, y: end.y - center.y };

  // 计算夹角（弧度）
  const dotProduct = startVec.x * endVec.x + startVec.y * endVec.y;
  const crossProduct = startVec.x * endVec.y - startVec.y * endVec.x;
  let angle = Math.atan2(crossProduct, dotProduct);

  if (isClockwise) {
    // 顺时针：确保角度为负（若为正，则减去2π）
    if (angle > 0) angle -= 2 * Math.PI;
  } else {
    // 逆时针：确保角度为正（若为负，则加上2π）
    if (angle < 0) angle += 2 * Math.PI;
  }

  // 计算弧长 = 半径 × |夹角|（弧度）
  return radius * Math.abs(angle);
}

/** 寻找最短路径 */
export function findShortestPath(
  edges: (LineRoute | ArcRoute)[],
  startId: string,
  endId: string
): (LineRoute | ArcRoute)[] | null {
  // Build adjacency list (bidirectional), including edge references and direction
  const adjacencyList: Record<
    string,
    Array<{
      neighbor: string;
      distance: number;
      edge: LineRoute | ArcRoute;
      isReversed: boolean;
    }>
  > = {};

  edges.forEach((edge) => {
    const { start, end, distance } = edge;
    if (!start.id || !end.id) {
      return; // Skip edges without valid IDs
    }

    // Add forward edge (start -> end)
    if (!adjacencyList[start.id]) {
      adjacencyList[start.id] = [];
    }
    adjacencyList[start.id].push({
      neighbor: end.id,
      distance,
      edge,
      isReversed: false, // Original direction
    });

    // Add reverse edge (end -> start) since lines/arcs are bidirectional
    if (!adjacencyList[end.id]) {
      adjacencyList[end.id] = [];
    }
    adjacencyList[end.id].push({
      neighbor: start.id,
      distance,
      edge,
      isReversed: true, // Reversed direction
    });
  });

  // Dijkstra's algorithm
  const distances: Record<string, number> = {};
  const previousNodes: Record<
    string,
    {
      node: string | null;
      edge: LineRoute | ArcRoute | null;
      isReversed: boolean;
    }
  > = {};
  const priorityQueue: Array<{ node: string; distance: number }> = [];

  // Initialize distances
  Object.keys(adjacencyList).forEach((nodeId) => {
    distances[nodeId] = nodeId === startId ? 0 : Infinity;
    priorityQueue.push({ node: nodeId, distance: distances[nodeId] });
    previousNodes[nodeId] = { node: null, edge: null, isReversed: false };
  });

  // Process the priority queue (simplified, not using a real heap)
  while (priorityQueue.length > 0) {
    // Sort queue to simulate priority (not efficient, but works for small graphs)
    priorityQueue.sort((a, b) => a.distance - b.distance);
    const current = priorityQueue.shift()!;

    // Early exit if we reach the end node
    if (current.node === endId) {
      break;
    }

    // Skip if we already found a shorter path
    if (current.distance > distances[current.node]) {
      continue;
    }

    // Explore neighbors
    for (const neighbor of adjacencyList[current.node] || []) {
      const newDistance = distances[current.node] + neighbor.distance;
      if (newDistance < distances[neighbor.neighbor]) {
        distances[neighbor.neighbor] = newDistance;
        previousNodes[neighbor.neighbor] = {
          node: current.node,
          edge: neighbor.edge,
          isReversed: neighbor.isReversed,
        };
        priorityQueue.push({ node: neighbor.neighbor, distance: newDistance });
      }
    }
  }

  // Reconstruct path (from end to start)
  const pathEdges: (LineRoute | ArcRoute)[] = [];
  let currentNode: string | null = endId;

  while (currentNode !== null && currentNode !== startId) {
    const previous = previousNodes[currentNode] as {
      node: string | null;
      edge: LineRoute | ArcRoute | null;
      isReversed: boolean;
    };
    if (!previous || !previous.node || !previous.edge) {
      return null; // No path found
    }

    // Adjust edge direction based on isReversed
    const correctedEdge: LineRoute | ArcRoute = previous.isReversed
      ? {
          ...previous.edge,
          start: previous.edge.end,
          end: previous.edge.start,
        }
      : { ...previous.edge };

    pathEdges.unshift(correctedEdge);
    currentNode = previous.node;
  }

  // Check if a path exists
  if (pathEdges.length === 0 && startId !== endId) {
    return null; // No path found
  }

  return pathEdges;
}

/** 将一个有符号32位整数分解为两个高低16位有符号整数。用于发送指令时的坐标转换 */
export function splitInt32ToInt16s(value: number) {
  // 强制32位有符号
  value = value | 0;
  // 获取高16位（自动符号扩展）
  const xh = (value >> 16) & 0xffff;
  // 获取低16位
  const xl = value & 0xffff;
  return {
    high: xh > 0x7fff ? xh - 0x10000 : xh,
    low: xl > 0x7fff ? xl - 0x10000 : xl,
  };
}

/** 前进任务。自动添加"自旋"动作 */
export function agvForwardTask(
  currentAngle: number,
  edges: (LineRoute | ArcRoute)[]
) {
  const tasks = edges.reduce((result, edge, index) => {
    const sx = splitInt32ToInt16s(edge.start.x);
    const sy = splitInt32ToInt16s(edge.start.y);
    const startId = parseInt(edge.start.id ?? "0");
    const ex = splitInt32ToInt16s(edge.end.x);
    const ey = splitInt32ToInt16s(edge.end.y);
    const endId = parseInt(edge.end.id ?? "0");

    if (index > 0) {
      const finalAngle = calculateFinalAngle({ ...edges[index - 1] });
      const startAngle = calculateStartAngle({ ...edge });
      // 假设当前任务分步的执行角度与上个任务角度偏差过大, 自动添加"自旋"任务
      if (isAngleDeviationTooLarge(finalAngle, startAngle)) {
        result.push({
          offset1: 2, // 自旋
          offset2: agvKey.spinSpeed, // 限速
          offset3: getHeadingFrom({
            point: edge.start,
            angle: startAngle,
          }),
          offset5: getCargoSizeFrom({
            obstacleAreas: edge.start.settings?.obstacleAreas,
          }),
          offset6: sx.low, // 起点x
          offset7: sx.high,
          offset8: sy.low, // 起点y
          offset9: sy.high,
          offset10: sx.low, // 终点x
          offset11: sx.high,
          offset12: sy.low, // 终点y
          offset13: sy.high,
          offset18: 1, // mapId
          offset19: startId, // 起点id
          offset20: startId, // 终点id
        });
      }
    }
    // 直线、弯道：设置卷帘门、避障区域、是否绕障
    const routeSettings = edge.settings?.[`${startId}-${endId}`];
    const cargoSize = getCargoSizeFrom({
      isObstacleDetour: routeSettings?.isObstacleDetour,
      obstacleAreas: routeSettings?.obstacleAreas,
      doorId: routeSettings?.doorId,
    });
    if ("center" in edge && edge.center) {
      // 弯道任务
      const cx = splitInt32ToInt16s(edge.center.x);
      const cy = splitInt32ToInt16s(edge.center.y);
      result.push({
        offset1: 2, // 弯道
        offset2: routeSettings?.limitSpeed || agvKey.curveSpeed, // 限速
        offset3: -1, // -1.正向 -2.倒车
        offset4: edge.distance, // 距离
        offset5: cargoSize, // cargo_size
        offset6: sx.low, // 起点x
        offset7: sx.high,
        offset8: sy.low, // 起点y
        offset9: sy.high,
        offset10: ex.low, // 终点x
        offset11: ex.high,
        offset12: ey.low, // 终点y
        offset13: ey.high,
        offset14: cx.low, // 圆心x
        offset15: cx.high,
        offset16: cy.low, // 圆心y
        offset17: cy.high,
        offset18: 0, // map_id
        offset19: startId, // 起点id
        offset20: endId, // 终点id
      });
    } else {
      // 直线任务
      result.push({
        offset1: 1, // 直线
        offset2: routeSettings?.limitSpeed || agvKey.straightSpeed, // 限速
        offset3: -1, // -1.正向 -2.倒车
        offset4: edge.distance, // 距离
        offset5: cargoSize, // cargo_size
        offset6: sx.low, // 起点x
        offset7: sx.high,
        offset8: sy.low, // 起点y
        offset9: sy.high,
        offset10: ex.low, // 终点x
        offset11: ex.high,
        offset12: ey.low, // 终点y
        offset13: ey.high,
        offset18: 0, // map_id
        offset19: startId, // 起点id
        offset20: endId, // 终点id
      });
    }
    return result;
  }, new Array<SubTask>());

  // 假如当前实际角度与第一个任务的角度偏差过大，则在第一条添加"自旋"任务
  const firstTask = edges[0];
  const startAngle = calculateStartAngle({ ...firstTask });
  if (isAngleDeviationTooLarge(currentAngle, startAngle)) {
    const sx = splitInt32ToInt16s(firstTask.start.x);
    const sy = splitInt32ToInt16s(firstTask.start.y);
    const startId = parseInt(firstTask.start.id ?? "0");
    tasks.unshift({
      offset1: 2, // 自旋
      offset2: agvKey.spinSpeed, // 速度
      offset3: getHeadingFrom({
        point: firstTask.start,
        angle: startAngle,
      }),
      offset5: getCargoSizeFrom({
        obstacleAreas: firstTask.start.settings?.obstacleAreas,
      }),
      offset6: sx.low, // 起点x
      offset7: sx.high,
      offset8: sy.low, // 起点y
      offset9: sy.high,
      offset10: sx.low, // 终点x
      offset11: sx.high,
      offset12: sy.low, // 终点y
      offset13: sy.high,
      offset18: 1, // mapId
      offset19: startId, // 起点id
      offset20: startId, // 终点id
    });
  }

  return {
    tasks,
    finalAngle: calculateFinalAngle({ ...edges[edges.length - 1] }),
  };
}

/** 取卸货任务(后退)，自动添加"自旋"动作 */
export function agvPickAndDropTask(
  currentAngle: number,
  edge: LineRoute,
  isPick: boolean
) {
  const sx = splitInt32ToInt16s(edge.start.x);
  const sy = splitInt32ToInt16s(edge.start.y);
  const startId = parseInt(edge.start.id ?? "0");
  const ex = splitInt32ToInt16s(edge.end.x);
  const ey = splitInt32ToInt16s(edge.end.y);
  const endId = parseInt(edge.end.id ?? "0");

  const tasks: SubTask[] = [];
  const startAngle = calculateStartAngle({ ...edge, reverse: true });
  if (isAngleDeviationTooLarge(currentAngle, startAngle)) {
    tasks.unshift({
      offset1: 2, // 自旋
      offset2: agvKey.spinSpeed, // 速度
      offset3: getHeadingFrom({
        point: edge.start,
        angle: startAngle,
      }),
      offset5: getCargoSizeFrom({
        obstacleAreas: edge.start.settings?.obstacleAreas,
      }),
      offset6: sx.low, // 起点x
      offset7: sx.high,
      offset8: sy.low, // 起点y
      offset9: sy.high,
      offset10: sx.low, // 终点x
      offset11: sx.high,
      offset12: sy.low, // 终点y
      offset13: sy.high,
      offset18: 1, // mapId
      offset19: startId, // 起点id
      offset20: startId, // 终点id
    });
  }

  let middleId = startId;
  let middle = { x: edge.start.x, y: edge.start.y };
  let mx = sx;
  let my = sy;
  // 取货任务: 自动将agv移动到可以识别到托盘的距离
  if (isPick && edge.distance > agvKey.recognizableDistance) {
    const dx = edge.end.x - edge.start.x;
    const dy = edge.end.y - edge.start.y;
    const length = Math.sqrt(dx * dx + dy * dy);
    const ratio = (length - agvKey.recognizableDistance) / length;
    middle = {
      x: edge.start.x + dx * ratio,
      y: edge.start.y + dy * ratio,
    };
    mx = splitInt32ToInt16s(middle.x);
    my = splitInt32ToInt16s(middle.y);
    middleId = endId * 100; // 新建中间点id
    tasks.push({
      offset1: 1, // 直线
      offset2: 400, // 限速，这里使用慢一点的弯道速度
      offset3: -2, // -1.正向 -2.倒车
      offset4: calculateLineDistance({ ...edge.start }, middle), // 距离
      offset5: 255, // 关闭所有避障区域
      offset6: sx.low, // 起点x
      offset7: sx.high,
      offset8: sy.low, // 起点y
      offset9: sy.high,
      offset10: mx.low, // 终点x
      offset11: mx.high,
      offset12: my.low, // 终点y
      offset13: my.high,
      offset18: 0, // map_id
      offset19: startId, // 起点id
      offset20: middleId, // 终点id
    });
  }

  // 取货 | 卸货
  tasks.push({
    offset1: isPick ? 9 : 1, // 9.自调节 | 1.直线
    offset2: 400, // 限速，这里使用慢一点的弯道速度
    offset3: -2, // -1.正向 -2.倒车
    offset4: calculateLineDistance(middle, { ...edge.end }), // 距离
    offset5: 255, // 关闭所有避障区域
    offset6: mx.low, // 起点x
    offset7: mx.high,
    offset8: my.low, // 起点y
    offset9: my.high,
    offset10: ex.low, // 终点x
    offset11: ex.high,
    offset12: ey.low, // 终点y
    offset13: ey.high,
    offset14: isPick ? 1600 : 0, // 插取深度
    offset18: 0, // map_id
    offset19: middleId, // 起点id
    offset20: endId, // 终点id
  });

  // 举升任务
  tasks.push({
    offset1: 3, // 举升
    offset2: isPick ? agvKey.raiseHeight : agvKey.lowerHeight, // 举升高度
    offset3: agvKey.raiseSpeed, // 举升速度
    offset9: ex.low, // 终点x
    offset10: ex.high,
    offset11: ey.low, // 终点y
    offset12: ey.high,
    offset18: 0, // 地图id
    offset19: endId, // 终点id
  });

  return {
    tasks,
    finalAngle: calculateFinalAngle({ ...edge, reverse: true }),
  };
}

/** 举升任务 */
export function agvRaiseTask(
  height: number,
  position: { x: number; y: number }
) {
  const ex = splitInt32ToInt16s(position.x);
  const ey = splitInt32ToInt16s(position.y);
  return {
    offset1: 3, // 举升
    offset2: height, // 举升高度
    offset3: agvKey.raiseSpeed, // 举升速度
    offset9: ex.low, // 终点x
    offset10: ex.high,
    offset11: ey.low, // 终点y
    offset12: ey.high,
    offset18: 0, // 地图id
    offset19: 9999, // 终点id
  } as SubTask;
}

/** 充电 | 取消充电 */
export function agvChargeTask(
  chargeId: string,
  chargeAgs?: {
    angle: number;
    edges: (LineRoute | ArcRoute)[];
  }
) {
  const pId = parseInt(chargeId);
  if (!chargeAgs) {
    // 停止充电
    return [{ offset1: 4, offset2: 2, offset19: pId }];
  }
  const { angle, edges } = chargeAgs;
  // 1.前往充电点
  const tasks = agvForwardTask(angle, edges).tasks;
  // 2.最后一段: 限速，这里使用慢一点的弯道速度
  tasks[tasks.length - 1].offset2 = 250;
  // 3.开始充电
  tasks.push({
    offset1: 4,
    offset2: 1,
    offset4: 255, // 关闭所有避障区域
    offset19: pId,
  });
  return tasks;
}

/** 计算角度基函数 */
function calculateBaseAngle(args: {
  start: { x: number; y: number };
  center?: { x: number; y: number }; // 圆心，有值代表时弧线
  end: { x: number; y: number };
  useStartPoint: boolean; // true: 计算起始角度, false: 计算最终角度
}): number {
  const { start, center, end, useStartPoint } = args;

  let radians = 0;
  if (center) {
    // 计算起点和终点相对于圆心的向量
    const startVec = { x: start.x - center.x, y: start.y - center.y };
    const endVec = { x: end.x - center.x, y: end.y - center.y };

    // 计算夹角（弧度）
    const dotProduct = startVec.x * endVec.x + startVec.y * endVec.y;
    const crossProduct = startVec.x * endVec.y - startVec.y * endVec.x;
    const angle = Math.atan2(crossProduct, dotProduct);

    // 计算切线角度
    const tangentPoint = useStartPoint ? start : end;
    const tangentAngle = Math.atan2(
      tangentPoint.y - center.y,
      tangentPoint.x - center.x
    );

    if (angle > 0) {
      // 逆时针：切线方向为半径顺时针旋转90度
      radians = tangentAngle + Math.PI / 2;
    } else {
      // 顺时针：切线方向为半径逆时针旋转90度
      radians = tangentAngle - Math.PI / 2;
    }
  } else {
    // 计算直线的角度
    radians = Math.atan2(end.y - start.y, end.x - start.x);
  }

  return radians;
}

/** 计算直线或弧线的起始角度 */
function calculateStartAngle(args: {
  start: { x: number; y: number };
  center?: { x: number; y: number };
  end: { x: number; y: number };
  reverse?: boolean;
}): number {
  const radians = calculateBaseAngle({ ...args, useStartPoint: true });
  const degrees = (radians * 180) / Math.PI;
  return degreesLimitIn180(degrees + (args.reverse === true ? 180 : 0));
}

/** 计算直线或弧线的最终角度 */
function calculateFinalAngle(args: {
  start: { x: number; y: number };
  center?: { x: number; y: number };
  end: { x: number; y: number };
  reverse?: boolean;
}): number {
  const radians = calculateBaseAngle({ ...args, useStartPoint: false });
  const degrees = (radians * 180) / Math.PI;
  return degreesLimitIn180(degrees + (args.reverse === true ? 180 : 0));
}

function degreesLimitIn180(degrees: number) {
  if (degrees > 180) {
    return degrees - 360;
  } else if (degrees <= -180) {
    return degrees + 360;
  } else {
    return degrees;
  }
}

/** 判断角度偏差是否过大 */
function isAngleDeviationTooLarge(
  startAngle: number,
  finalAngle: number
): boolean {
  const diff = Math.abs(finalAngle - startAngle) % 360;
  const minAngleDiff = Math.min(diff, 360 - diff);
  return minAngleDiff > agvKey.angleRange;
}

/** 根据'避障区域'和'卷帘门编号'获取cargoSize */
function getCargoSizeFrom(args: {
  isObstacleDetour?: boolean;
  obstacleAreas?: number[];
  doorId?: number;
}) {
  const { isObstacleDetour = false, obstacleAreas = [], doorId = 0 } = args;
  // 1.高8位（其实是3-8位），用于标记卷帘门的编号。门的最大编号 id <= 63
  const high = doorId.toString(2).padStart(8, "0").split("");
  // 2.高第2位，用于标识是否绕障
  high[1] = isObstacleDetour ? "1" : "0";
  // 如果是绕障，就没必要传递避障区域，两者只能取其一
  if (isObstacleDetour) {
    return parseInt(`${high.join("")}${"".padStart(8, "0")}`, 2);
  }
  // 3.低8位，用于标识避障区域
  const low = [0, 0, 0, 0, 0, 0, 0, 0];
  for (const i of obstacleAreas) {
    low[7 - i] = 1;
  }
  return parseInt(`${high.join("")}${low.join("")}`, 2);
}

/** 获取自旋方向 */
function getHeadingFrom(args: { point: Point; angle: number }) {
  const { point, angle } = args;
  switch (point.settings?.rotationDirection) {
    case 1: // 往左
      return angle + 1000;

    case 2: // 往右
      return angle + 2000;

    default: // 无
      return angle;
  }
}
