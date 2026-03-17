import { useRef, useEffect } from "react";
import { useViewControl } from "./hooks/useViewControl";
import { useRendering } from "./hooks/useRendering";
import { useCanvas } from "./hooks/useCanvas";
import { ToolBar } from "./components/ToolBar";
import { StatusBar } from "./components/StatusBar";
import { HelpPanel } from "./components/HelpPanel";

const MapCanvasEditor = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // 画布尺寸管理
  const { canvasSize } = useCanvas({
    containerRef,
    canvasRef,
  });

  // 视图控制相关
  const {
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
  } = useViewControl({
    canvasRef,
  });

  // 渲染相关
  const { draw, getUnit } = useRendering({
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
    selectedPoints,
    selectedLines,
    isBoxSelecting,
    boxSelectStart,
    boxSelectEnd,
    applyConstraints,
  });

  // 重绘
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (ctx) {
      draw(ctx);
    }
  }, [draw]);

  return (
    <div className="bg-white rounded-lg shadow-lg h-full flex flex-col overflow-hidden">
      {/* 工具栏 */}
      <ToolBar
        drawMode={drawMode}
        onSelectMode={switchDrawMode}
        constraintOptions={constraintOptions}
        setConstraintOptions={setConstraintOptions}
        scale={scale}
        setScale={setScale}
        resetView={resetView}
        clearCanvas={clearCanvas}
        getUnit={getUnit}
      />

      {/* 状态栏 */}
      <StatusBar
        drawMode={drawMode}
        isDrawing={isDrawing}
        points={points}
        lines={lines}
        snapPoint={snapPoint}
        mousePos={mousePos}
      />

      {/* 画布容器 */}
      <div
        ref={containerRef}
        className="flex-1 relative overflow-hidden bg-gray-100 w-full min-h-0"
      >
        <canvas
          ref={canvasRef}
          width={canvasSize.width}
          height={canvasSize.height}
          className="w-full h-full"
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onWheel={handleWheel}
          style={{
            cursor: isDragging
              ? "grabbing"
              : drawMode === "none" || drawMode === "select"
              ? "default"
              : "crosshair",
          }}
        />

        {/* 帮助提示 */}
        <HelpPanel />
      </div>
    </div>
  );
};

export default MapCanvasEditor;
