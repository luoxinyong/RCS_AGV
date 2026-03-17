import { useState, useCallback, useEffect, useRef, RefObject } from "react";
import { ConstraintOptions, DrawMode, Line, Point } from "../types";
import { screenToWorld } from "../utils";

interface UseViewControlProps {
  canvasRef: RefObject<HTMLCanvasElement>;
}

export const useViewControl = ({ canvasRef }: UseViewControlProps) => {
  // 视图控制
  const [scale, setScale] = useState(50);
  const [offset, setOffset] = useState({ x: 400, y: 300 });
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });

  // 基础状态
  const [points, setPoints] = useState<Point[]>([]);
  const [lines, setLines] = useState<Line[]>([]);

  // 选择状态
  const [selectedPoints, setSelectedPoints] = useState<number[]>([]);
  const [selectedLines, setSelectedLines] = useState<number[]>([]);

  // 框选状态
  const [isBoxSelecting, setIsBoxSelecting] = useState(false);
  const [boxSelectStart, setBoxSelectStart] = useState<{
    x: number;
    y: number;
  } | null>(null);
  const [boxSelectEnd, setBoxSelectEnd] = useState<{
    x: number;
    y: number;
  } | null>(null);

  // 绘制状态
  const [drawMode, setDrawMode] = useState<DrawMode>("none");
  const [constraintOptions, setConstraintOptions] = useState<ConstraintOptions>(
    {
      isOrthogonal: false,
      isFixedLength: false,
      fixedLength: 1000, // 默认1米，单位毫米
    }
  );
  const [isDrawing, setIsDrawing] = useState(false);
  const [currentStartPoint, setCurrentStartPoint] = useState<Point | null>(
    null
  );
  const [currentMiddlePoint, setCurrentMiddlePoint] = useState<Point | null>(
    null
  );
  const [arcStep, setArcStep] = useState(0); // 0: 起点, 1: 中间点, 2: 终点

  // 鼠标状态
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });
  const [snapPoint, setSnapPoint] = useState<Point | null>(null);
  const debounceTimeoutRef = useRef<NodeJS.Timeout>();

  // 绘制设置
  const snapDistance = 10; // 吸附距离像素

  // ID计数器
  const [nextPointId, setNextPointId] = useState(1);
  const [nextLineId, setNextLineId] = useState(1);

  // 吸附点检测
  const findSnapPoint = useCallback(
    (worldPos: { x: number; y: number }) => {
      const snapDistanceWorld = snapDistance / scale;

      // 检查现有点
      for (const point of points) {
        const distance = Math.sqrt(
          Math.pow(point.x - worldPos.x, 2) + Math.pow(point.y - worldPos.y, 2)
        );
        if (distance < snapDistanceWorld) {
          return point;
        }
      }

      // 检查线段上的点
      for (const line of lines) {
        if (line.type === "straight") {
          const t = getClosestPointOnLine(
            worldPos,
            line.startPoint,
            line.endPoint
          );
          if (t >= 0 && t <= 1) {
            const closestPoint = {
              x: line.startPoint.x + t * (line.endPoint.x - line.startPoint.x),
              y: line.startPoint.y + t * (line.endPoint.y - line.startPoint.y),
            };
            const distance = Math.sqrt(
              Math.pow(closestPoint.x - worldPos.x, 2) +
                Math.pow(closestPoint.y - worldPos.y, 2)
            );
            if (distance < snapDistanceWorld) {
              return { id: -1, ...closestPoint };
            }
          }
        }
      }

      return null;
    },
    [points, lines, scale]
  );

  // 帮助函数 - 从hook移出
  const getClosestPointOnLine = (
    point: { x: number; y: number },
    lineStart: Point,
    lineEnd: Point
  ) => {
    const A = point.x - lineStart.x;
    const B = point.y - lineStart.y;
    const C = lineEnd.x - lineStart.x;
    const D = lineEnd.y - lineStart.y;

    const dot = A * C + B * D;
    const lenSq = C * C + D * D;

    return lenSq !== 0 ? dot / lenSq : -1;
  };

  // 防抖的鼠标移动处理
  const debouncedFindSnap = useCallback(
    (worldPos: { x: number; y: number }) => {
      if (debounceTimeoutRef.current) {
        clearTimeout(debounceTimeoutRef.current);
      }
      debounceTimeoutRef.current = setTimeout(() => {
        const snap = findSnapPoint(worldPos);
        setSnapPoint(snap);
      }, 16);
    },
    [findSnapPoint]
  );

  // 组件卸载时清理定时器
  useEffect(() => {
    return () => {
      if (debounceTimeoutRef.current) {
        clearTimeout(debounceTimeoutRef.current);
      }
    };
  }, []);

  // 应用约束
  const applyConstraints = useCallback(
    (startPoint: Point, endPoint: Point) => {
      if (drawMode === "none") {
        return endPoint;
      }

      if (constraintOptions.isOrthogonal) {
        const deltaX = Math.abs(endPoint.x - startPoint.x);
        const deltaY = Math.abs(endPoint.y - startPoint.y);

        if (deltaX > deltaY) {
          return { ...endPoint, y: startPoint.y }; // 水平线
        } else {
          return { ...endPoint, x: startPoint.x }; // 垂直线
        }
      }

      if (constraintOptions.isFixedLength) {
        const dx = endPoint.x - startPoint.x;
        const dy = endPoint.y - startPoint.y;
        const currentLength = Math.sqrt(dx * dx + dy * dy);
        const targetLength = constraintOptions.fixedLength / 1000; // 转换为米

        if (currentLength === 0) return endPoint;

        const ratio = targetLength / currentLength;
        return {
          id: endPoint.id,
          x: startPoint.x + dx * ratio,
          y: startPoint.y + dy * ratio,
        };
      }

      return endPoint;
    },
    [constraintOptions, drawMode]
  );

  // 创建或获取点
  const createOrGetPoint = useCallback(
    (worldPos: { x: number; y: number }) => {
      if (snapPoint && snapPoint.id !== -1) {
        return snapPoint; // 复用已有点
      }

      const newPoint: Point = {
        id: nextPointId,
        x: snapPoint ? snapPoint.x : worldPos.x,
        y: snapPoint ? snapPoint.y : worldPos.y,
      };

      if (snapPoint && snapPoint.id === -1) {
        // 在线段上创建新点
        setPoints((prev) => [...prev, newPoint]);
      } else if (!snapPoint) {
        // 创建全新点
        setPoints((prev) => [...prev, newPoint]);
      }

      setNextPointId((prev) => prev + 1);
      return newPoint;
    },
    [snapPoint, nextPointId]
  );

  // 完成框选
  const completeBoxSelection = useCallback(() => {
    if (boxSelectStart && boxSelectEnd) {
      // 找出相关的坐标点
      const minX = Math.min(boxSelectStart.x, boxSelectEnd.x);
      const maxX = Math.max(boxSelectStart.x, boxSelectEnd.x);
      const minY = Math.min(boxSelectStart.y, boxSelectEnd.y);
      const maxY = Math.max(boxSelectStart.y, boxSelectEnd.y);
      const selectedIds = points
        .filter(
          (point) =>
            point.x >= minX &&
            point.x <= maxX &&
            point.y >= minY &&
            point.y <= maxY
        )
        .map((point) => point.id);
      setSelectedPoints(selectedIds);

      // 找出相关的线段
      const relatedLines = lines
        .filter(
          (line) =>
            selectedIds.includes(line.startPoint.id) &&
            selectedIds.includes(line.endPoint.id)
        )
        .map((line) => line.id);
      setSelectedLines(relatedLines);
    }

    setIsBoxSelecting(false);
    setBoxSelectStart(null);
    setBoxSelectEnd(null);
    // setDrawMode("select"); // 框选后切换到选择模式
  }, [boxSelectStart, boxSelectEnd, lines, points]);

  // 删除选中的元素
  const deleteSelected = useCallback(() => {
    if (selectedPoints.length === 0 && selectedLines.length === 0) {
      return; // 没有选中任何元素
    }

    if (selectedLines.length > 0) {
      // 删除选中的线
      setLines((prev) =>
        prev.filter((line) => !selectedLines.includes(line.id))
      );

      // 找出没有连接其他线的点
      const remainingLines = lines.filter(
        (line) => !selectedLines.includes(line.id)
      );
      const connectedPoints = new Set<number>();

      remainingLines.forEach((line) => {
        connectedPoints.add(line.startPoint.id);
        connectedPoints.add(line.endPoint.id);
        if (line.middlePoint) {
          connectedPoints.add(line.middlePoint.id);
        }
      });

      // 删除没有连接的点
      setPoints((prev) =>
        prev.filter(
          (point) =>
            connectedPoints.has(point.id) || !selectedPoints.includes(point.id)
        )
      );
    } else if (selectedPoints.length > 0) {
      // 删除与选中点关联的线
      const connectedLines = lines.filter(
        (line) =>
          selectedPoints.includes(line.startPoint.id) ||
          selectedPoints.includes(line.endPoint.id) ||
          (line.middlePoint && selectedPoints.includes(line.middlePoint.id))
      );

      setLines((prev) =>
        prev.filter((line) => !connectedLines.some((cl) => cl.id === line.id))
      );

      // 删除选中的点
      setPoints((prev) =>
        prev.filter((point) => !selectedPoints.includes(point.id))
      );
    }

    // 清空选择
    setSelectedPoints([]);
    setSelectedLines([]);
  }, [selectedPoints, selectedLines, lines]);

  // 切换绘制模式
  const switchDrawMode = useCallback((mode: DrawMode) => {
    setDrawMode(mode);
    setArcStep(0);
    setIsDrawing(false);
    setSelectedPoints([]);
    setSelectedLines([]);

    // 清除框选状态
    if (mode !== "boxSelect") {
      setIsBoxSelecting(false);
      setBoxSelectStart(null);
      setBoxSelectEnd(null);
    }
  }, []);

  // 清空画布
  const clearCanvas = useCallback(() => {
    setPoints([]);
    setLines([]);
    setNextPointId(1);
    setNextLineId(1);
    setCurrentStartPoint(null);
    setCurrentMiddlePoint(null);
    switchDrawMode("none");
  }, [switchDrawMode]);

  // 重置视图
  const resetView = useCallback(() => {
    setScale(50);
    const width = canvasRef.current?.width ?? 0;
    const height = canvasRef.current?.height ?? 0;
    setOffset({ x: width / 2, y: height / 2 });
  }, [canvasRef]);

  // 处理拖拽开始
  const handleDragStart = useCallback(
    (e: React.MouseEvent) => {
      setIsDragging(true);
      setDragStart({ x: e.clientX - offset.x, y: e.clientY - offset.y });
    },
    [offset]
  );

  // 处理拖拽移动
  const handleDragMove = useCallback(
    (e: React.MouseEvent) => {
      if (!isDragging) return;

      setOffset({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    },
    [isDragging, dragStart]
  );

  // 处理拖拽结束
  const handleDragEnd = useCallback(() => {
    setIsDragging(false);
  }, []);

  // 鼠标事件处理
  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (e.button === 1) {
        // 中键拖拽
        handleDragStart(e);
        return;
      }

      const rect = canvasRef.current?.getBoundingClientRect();
      if (e.button !== 0 || !rect) return;

      const worldPos = screenToWorld(
        { x: e.clientX, y: e.clientY },
        offset,
        rect,
        scale
      );

      // 处理框选模式
      if (drawMode === "boxSelect") {
        setIsBoxSelecting(true);
        setBoxSelectStart(worldPos);
        setBoxSelectEnd(worldPos);
        return;
      }

      if (drawMode === "none") return;

      if (drawMode === "straight") {
        if (!isDrawing) {
          // 开始绘制直线
          const startPoint = createOrGetPoint(worldPos);
          setCurrentStartPoint(startPoint);
          setIsDrawing(true);
        } else {
          // 完成直线绘制
          if (currentStartPoint) {
            let endPoint = createOrGetPoint(worldPos);
            endPoint = applyConstraints(currentStartPoint, endPoint);

            const newLine = {
              id: nextLineId,
              startPoint: currentStartPoint,
              endPoint: endPoint,
              type: "straight" as const,
              fixedLength: constraintOptions.isFixedLength
                ? constraintOptions.fixedLength
                : undefined,
            };

            setLines((prev) => [...prev, newLine]);
            setNextLineId((prev) => prev + 1);
          }

          setIsDrawing(false);
          setCurrentStartPoint(null);
        }
      }

      if (drawMode === "arc") {
        if (arcStep === 0) {
          // 设置起点
          const startPoint = createOrGetPoint(worldPos);
          setCurrentStartPoint(startPoint);
          setArcStep(1);
          setIsDrawing(true);
        } else if (arcStep === 1) {
          // 设置中间点
          const middlePoint = createOrGetPoint(worldPos);
          setCurrentMiddlePoint(middlePoint);
          setArcStep(2);
        } else if (arcStep === 2) {
          // 完成弧线绘制
          if (currentStartPoint && currentMiddlePoint) {
            const endPoint = createOrGetPoint(worldPos);

            const newLine = {
              id: nextLineId,
              startPoint: currentStartPoint,
              endPoint: endPoint,
              middlePoint: currentMiddlePoint,
              type: "arc" as const,
            };

            setLines((prev) => [...prev, newLine]);
            setNextLineId((prev) => prev + 1);
          }

          setIsDrawing(false);
          setCurrentStartPoint(null);
          setCurrentMiddlePoint(null);
          setArcStep(0);
        }
      }
    },
    [
      canvasRef,
      drawMode,
      offset,
      scale,
      handleDragStart,
      isDrawing,
      createOrGetPoint,
      currentStartPoint,
      applyConstraints,
      nextLineId,
      constraintOptions,
      arcStep,
      currentMiddlePoint,
    ]
  );

  const handleMouseMove = useCallback(
    (e: React.MouseEvent) => {
      if (isDragging) {
        handleDragMove(e);
        return;
      }

      const rect = canvasRef.current?.getBoundingClientRect();
      if (!rect) {
        return;
      }
      const worldPos = screenToWorld(
        { x: e.clientX, y: e.clientY },
        offset,
        rect,
        scale
      );
      setMousePos(worldPos);
      debouncedFindSnap(worldPos);

      // 处理框选更新
      if (isBoxSelecting && boxSelectStart) {
        setBoxSelectEnd(worldPos);
      }
    },
    [
      canvasRef,
      isDragging,
      offset,
      scale,
      debouncedFindSnap,
      handleDragMove,
      isBoxSelecting,
      boxSelectStart,
    ]
  );

  const handleMouseUp = useCallback(() => {
    handleDragEnd();

    // 处理框选完成
    if (isBoxSelecting) {
      completeBoxSelection();
    }
  }, [handleDragEnd, isBoxSelecting, completeBoxSelection]);

  // 处理滚轮缩放
  const handleWheel = useCallback(
    (e: React.WheelEvent) => {
      e.preventDefault();
      const rect = e.currentTarget.getBoundingClientRect();
      const canvasRect = canvasRef.current?.getBoundingClientRect();
      if (!canvasRect || !rect) return;

      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const worldBefore = screenToWorld(
        { x: e.clientX, y: e.clientY },
        offset,
        canvasRect,
        scale
      );
      const scaleFactor = e.deltaY > 0 ? 0.9 : 1.1;
      const newScale = Math.max(0.1, Math.min(10000, scale * scaleFactor));

      setScale(newScale);

      const worldAfter = {
        x: (mouseX - offset.x) / newScale,
        y: (mouseY - offset.y) / newScale,
      };

      setOffset((prev) => ({
        x: prev.x + (worldAfter.x - worldBefore.x) * newScale,
        y: prev.y + (worldAfter.y - worldBefore.y) * newScale,
      }));
    },
    [canvasRef, scale, offset]
  );

  // 键盘事件
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        setCurrentStartPoint(null);
        setCurrentMiddlePoint(null);
        switchDrawMode("none");
      } else if (e.key === "Delete" || e.key === "Backspace") {
        deleteSelected(); // 删除选中的点和线
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [
    setCurrentStartPoint,
    setCurrentMiddlePoint,
    switchDrawMode,
    selectedPoints,
    selectedLines,
    deleteSelected,
  ]);

  return {
    scale,
    setScale,
    offset,
    isDragging,
    points,
    lines,
    snapPoint,
    isDrawing,
    drawMode,
    currentStartPoint,
    currentMiddlePoint,
    mousePos,
    arcStep,
    constraintOptions,
    setConstraintOptions,
    selectedPoints,
    selectedLines,
    isBoxSelecting,
    boxSelectStart,
    boxSelectEnd,
    resetView,
    handleWheel,
    handleMouseUp,
    handleMouseMove,
    handleMouseDown,
    applyConstraints,
    clearCanvas,
    switchDrawMode,
  };
};
