import { useState, useEffect, RefObject } from "react";

interface UseCanvasProps {
  containerRef: RefObject<HTMLDivElement>;
  canvasRef: RefObject<HTMLCanvasElement>;
}

export const useCanvas = ({ containerRef, canvasRef }: UseCanvasProps) => {
  // 画布尺寸状态
  const [canvasSize, setCanvasSize] = useState({ width: 800, height: 600 });

  // Canvas尺寸自适应
  useEffect(() => {
    const handleResize = () => {
      const rect = containerRef.current?.getBoundingClientRect();
      if (rect) {
        setCanvasSize({
          width: rect.width,
          height: rect.height,
        });
      }
    };

    handleResize();
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, [containerRef]);

  // 更新Canvas尺寸
  useEffect(() => {
    if (canvasRef.current) {
      canvasRef.current.width = canvasSize.width;
      canvasRef.current.height = canvasSize.height;
    }
  }, [canvasSize, canvasRef]);

  return {
    canvasSize,
  };
};
