import { useCallback } from "react";
import { Point, Line, DrawMode, UnitInfo } from "../types";
import { createArcFromThreePoints, worldToScreen } from "../utils";

interface UseRenderingProps {
  scale: number;
  offset: { x: number; y: number };
  points: Point[];
  lines: Line[];
  snapPoint: Point | null;
  isDrawing: boolean;
  drawMode: DrawMode;
  currentStartPoint: Point | null;
  currentMiddlePoint: Point | null;
  mousePos: { x: number; y: number };
  arcStep: number;
  // 选择状态
  selectedPoints?: number[];
  selectedLines?: number[];
  hoveredPoint?: number | null;
  hoveredLine?: number | null;
  // 框选状态
  isBoxSelecting?: boolean;
  boxSelectStart?: { x: number; y: number } | null;
  boxSelectEnd?: { x: number; y: number } | null;
  applyConstraints: (startPoint: Point, endPoint: Point) => Point;
}

export const useRendering = ({
  scale,
  offset,
  points,
  lines,
  snapPoint,
  isDrawing,
  drawMode,
  currentStartPoint,
  currentMiddlePoint,
  mousePos,
  arcStep,
  selectedPoints = [],
  selectedLines = [],
  hoveredPoint = null,
  hoveredLine = null,
  isBoxSelecting = false,
  boxSelectStart = null,
  boxSelectEnd = null,
  applyConstraints,
}: UseRenderingProps) => {
  const getUnit = useCallback((): UnitInfo => {
    if (scale >= 1000) return { unit: "mm", factor: 1000, step: 1 };
    if (scale >= 100) return { unit: "cm", factor: 100, step: 10 };
    if (scale >= 10) return { unit: "m", factor: 1, step: 1000 };
    return { unit: "10m", factor: 0.1, step: 10000 };
  }, [scale]);

  // 绘制网格
  const drawGrid = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      const { width, height } = ctx.canvas;
      const { unit, factor, step } = getUnit();

      ctx.strokeStyle = "#e8e8e8";
      ctx.lineWidth = 1;

      // 计算基础网格间距
      const baseGridSpacing = scale * (step / 1000);

      // 动态调整间距以限制刻度数量
      const maxTicks = 10;

      // 计算x轴需要的倍数来限制刻度数量
      let xMultiplier = 1;
      while (width / (baseGridSpacing * xMultiplier) > maxTicks) {
        xMultiplier++;
      }

      // 计算y轴需要的倍数来限制刻度数量
      let yMultiplier = 1;
      while (height / (baseGridSpacing * yMultiplier) > maxTicks) {
        yMultiplier++;
      }

      const xGridSpacing = baseGridSpacing * xMultiplier;
      const yGridSpacing = baseGridSpacing * yMultiplier;

      // 绘制垂直网格线
      let tickCount = 0;
      for (
        let x = offset.x % xGridSpacing;
        x < width && tickCount < maxTicks;
        x += xGridSpacing
      ) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();

        const worldX = (x - offset.x) / scale;
        ctx.fillStyle = "#666";
        ctx.font = "10px Arial";
        ctx.fillText(
          `${(worldX * factor * xMultiplier).toFixed(1)}${unit}`,
          x + 2,
          12
        );
        tickCount++;
      }

      // 绘制水平网格线
      tickCount = 0;
      for (
        let y = offset.y % yGridSpacing;
        y < height && tickCount < maxTicks;
        y += yGridSpacing
      ) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();

        const worldY = (y - offset.y) / scale;
        ctx.fillStyle = "#666";
        ctx.font = "10px Arial";
        ctx.fillText(
          `${(worldY * factor * yMultiplier).toFixed(1)}${unit}`,
          2,
          y - 2
        );
        tickCount++;
      }
    },
    [getUnit, scale, offset]
  );

  // 绘制直线
  const drawStraightLine = useCallback(
    (ctx: CanvasRenderingContext2D, line: Line) => {
      const start = worldToScreen(line.startPoint, offset, scale);
      const end = worldToScreen(line.endPoint, offset, scale);
      const isSelected = selectedLines.includes(line.id);
      const isHovered = hoveredLine === line.id;
      // 高亮连接点
      const isConnectedToHovered =
        isHovered &&
        (line.startPoint.id === hoveredPoint ||
          line.endPoint.id === hoveredPoint);

      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);

      // 根据状态确定线条样式
      if (isSelected) {
        ctx.strokeStyle = "#1890ff"; // antd主题蓝色
        ctx.lineWidth = 3;
      } else if (isHovered) {
        ctx.strokeStyle = "#40a9ff"; // antd浅蓝色
        ctx.lineWidth = 2.5;
      } else if (isConnectedToHovered) {
        ctx.strokeStyle = "#69c0ff"; // 更浅的蓝色
        ctx.lineWidth = 2.5;
      } else {
        ctx.strokeStyle = "#007bff";
        ctx.lineWidth = 2;
      }

      ctx.stroke();

      // 绘制长度标注
      const midX = (start.x + end.x) / 2;
      const midY = (start.y + end.y) / 2;
      const length =
        Math.sqrt(
          Math.pow(line.endPoint.x - line.startPoint.x, 2) +
            Math.pow(line.endPoint.y - line.startPoint.y, 2)
        ) * 1000; // 转换为毫米

      ctx.fillStyle = isSelected || isHovered ? "#1890ff" : "#333";
      ctx.font = "12px Arial";
      ctx.fillText(`${length.toFixed(0)}mm`, midX + 5, midY - 5);
    },
    [offset, scale, selectedLines, hoveredLine, hoveredPoint]
  );

  // 绘制弧线
  const drawArcLine = useCallback(
    (ctx: CanvasRenderingContext2D, line: Line) => {
      if (!line.middlePoint) return;

      const arcData = createArcFromThreePoints(
        line.startPoint,
        line.middlePoint,
        line.endPoint
      );

      if (!arcData) return;

      const center = worldToScreen(
        { x: arcData.centerX, y: arcData.centerY },
        offset,
        scale
      );
      const radiusScreen = arcData.radius * scale;
      const isSelected = selectedLines.includes(line.id);
      const isHovered = hoveredLine === line.id;

      ctx.beginPath();
      ctx.arc(
        center.x,
        center.y,
        radiusScreen,
        arcData.startAngle,
        arcData.endAngle
      );

      // 根据状态确定线条样式
      if (isSelected) {
        ctx.strokeStyle = "#1890ff"; // antd主题蓝色
        ctx.lineWidth = 3;
      } else if (isHovered) {
        ctx.strokeStyle = "#40a9ff"; // antd浅蓝色
        ctx.lineWidth = 2.5;
      } else {
        ctx.strokeStyle = "#007bff";
        ctx.lineWidth = 2;
      }

      ctx.stroke();
    },
    [offset, scale, selectedLines, hoveredLine]
  );

  // 绘制点
  const drawPoint = useCallback(
    (ctx: CanvasRenderingContext2D, point: Point) => {
      const screen = worldToScreen(point, offset, scale);
      const isSelected = selectedPoints.includes(point.id);
      const isHovered = hoveredPoint === point.id;
      const radius = isSelected || isHovered ? 6 : 4;

      ctx.beginPath();
      ctx.arc(screen.x, screen.y, radius, 0, 2 * Math.PI);

      // 根据状态确定填充色
      if (isSelected) {
        ctx.fillStyle = "#1890ff"; // antd主题蓝色
      } else if (isHovered) {
        ctx.fillStyle = "#40a9ff"; // antd浅蓝色
      } else {
        ctx.fillStyle = "#ff6b6b";
      }

      ctx.fill();
      ctx.strokeStyle = isSelected ? "#096dd9" : "#fff";
      ctx.lineWidth = isSelected || isHovered ? 2.5 : 2;
      ctx.stroke();

      ctx.fillStyle = "#333";
      ctx.font = "10px Arial";
      ctx.fillText(`P${point.id}`, screen.x + 6, screen.y - 6);
    },
    [offset, scale, selectedPoints, hoveredPoint]
  );

  // 绘制预览
  const drawPreview = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      if (!isDrawing) return;

      ctx.setLineDash([5, 5]);
      ctx.strokeStyle = "#999";
      ctx.lineWidth = 1;

      if (drawMode === "straight" && currentStartPoint) {
        const start = worldToScreen(currentStartPoint, offset, scale);
        let endPoint = snapPoint || { id: -1, x: mousePos.x, y: mousePos.y };

        // 应用约束
        endPoint = applyConstraints(currentStartPoint, endPoint);
        const end = worldToScreen(endPoint, offset, scale);

        ctx.beginPath();
        ctx.moveTo(start.x, start.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();

        // 显示长度预览
        const length =
          Math.sqrt(
            Math.pow(endPoint.x - currentStartPoint.x, 2) +
              Math.pow(endPoint.y - currentStartPoint.y, 2)
          ) * 1000;

        ctx.fillStyle = "#666";
        ctx.font = "12px Arial";
        ctx.fillText(`${length.toFixed(0)}mm`, end.x + 5, end.y - 5);
      }

      if (drawMode === "arc") {
        if (arcStep === 1 && currentStartPoint) {
          // 预览从起点到当前鼠标位置的线
          const start = worldToScreen(currentStartPoint, offset, scale);
          const current = worldToScreen(mousePos, offset, scale);

          ctx.beginPath();
          ctx.moveTo(start.x, start.y);
          ctx.lineTo(current.x, current.y);
          ctx.stroke();
        } else if (arcStep === 2 && currentStartPoint && currentMiddlePoint) {
          // 预览弧线
          const endPoint = snapPoint || {
            id: -1,
            x: mousePos.x,
            y: mousePos.y,
          };
          const arcData = createArcFromThreePoints(
            currentStartPoint,
            currentMiddlePoint,
            endPoint
          );

          if (arcData) {
            const center = worldToScreen(
              { x: arcData.centerX, y: arcData.centerY },
              offset,
              scale
            );
            const radiusScreen = arcData.radius * scale;

            ctx.beginPath();
            ctx.arc(
              center.x,
              center.y,
              radiusScreen,
              arcData.startAngle,
              arcData.endAngle
            );
            ctx.stroke();
          }
        }
      }

      ctx.setLineDash([]);
    },
    [
      isDrawing,
      drawMode,
      currentStartPoint,
      currentMiddlePoint,
      mousePos,
      snapPoint,
      applyConstraints,
      arcStep,
      scale,
      offset,
    ]
  );

  // 绘制框选区域
  const drawBoxSelection = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      if (!isBoxSelecting || !boxSelectStart || !boxSelectEnd) return;

      const start = worldToScreen(boxSelectStart, offset, scale);
      const end = worldToScreen(boxSelectEnd, offset, scale);

      const width = end.x - start.x;
      const height = end.y - start.y;

      // 绘制半透明的选择框
      ctx.fillStyle = "rgba(24, 144, 255, 0.1)"; // 浅蓝色半透明
      ctx.fillRect(start.x, start.y, width, height);

      // 绘制边框
      ctx.strokeStyle = "#1890ff";
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 5]);
      ctx.strokeRect(start.x, start.y, width, height);
      ctx.setLineDash([]);
    },
    [isBoxSelecting, boxSelectStart, boxSelectEnd, offset, scale]
  );

  // 主绘制函数
  const draw = useCallback(
    (ctx: CanvasRenderingContext2D) => {
      if (!ctx) return;

      const canvas = ctx.canvas;
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // 绘制网格
      drawGrid(ctx);

      // 绘制已有的线
      lines.forEach((line) => {
        if (line.type === "straight") {
          drawStraightLine(ctx, line);
        } else {
          drawArcLine(ctx, line);
        }
      });

      // 绘制点
      points.forEach((point) => drawPoint(ctx, point));

      // 绘制吸附点
      if (snapPoint) {
        const screen = worldToScreen(snapPoint, offset, scale);
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, 8, 0, 2 * Math.PI);
        ctx.strokeStyle = "#00ff00";
        ctx.lineWidth = 2;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 绘制框选区域
      drawBoxSelection(ctx);

      // 绘制预览
      drawPreview(ctx);
    },
    [
      drawGrid,
      lines,
      points,
      snapPoint,
      drawBoxSelection,
      drawPreview,
      drawStraightLine,
      drawArcLine,
      drawPoint,
      offset,
      scale,
    ]
  );

  return {
    draw,
    getUnit,
  };
};
