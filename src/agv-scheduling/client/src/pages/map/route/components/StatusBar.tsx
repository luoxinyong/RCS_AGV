import { Tag, Space } from "antd";
import { Point, Line, DrawMode } from "../types";

interface StatusBarProps {
  drawMode: DrawMode;
  isDrawing: boolean;
  points: Point[];
  lines: Line[];
  snapPoint: Point | null;
  mousePos: { x: number; y: number };
  selectedPoints?: number[];
  selectedLines?: number[];
}

export const StatusBar = ({
  drawMode,
  isDrawing,
  points,
  lines,
  snapPoint,
  mousePos,
  selectedPoints = [],
  selectedLines = [],
}: StatusBarProps) => {
  // 获取当前模式对应的标签类型
  const getModeTagType = () => {
    switch (drawMode) {
      case "straight":
        return "blue";
      case "arc":
        return "purple";
      case "select":
        return "green";
      case "boxSelect":
        return "cyan";
      default:
        return "default";
    }
  };

  // 获取模式显示文本
  const getModeText = () => {
    switch (drawMode) {
      case "straight":
        return "直线";
      case "arc":
        return "弧线";
      case "select":
        return "选择";
      case "boxSelect":
        return "框选";
      default:
        return "无";
    }
  };

  return (
    <div className="bg-white border-b px-4 py-2 text-sm flex items-center justify-between">
      <Space size="middle">
        <Space size="small">
          <span>模式:</span>
          <Tag color={getModeTagType()}>{getModeText()}</Tag>
        </Space>

        <Space size="small">
          <span>点数: {points.length}</span>
          {selectedPoints.length > 0 && (
            <Tag color="blue">已选: {selectedPoints.length}</Tag>
          )}
        </Space>

        <Space size="small">
          <span>线数: {lines.length}</span>
          {selectedLines.length > 0 && (
            <Tag color="blue">已选: {selectedLines.length}</Tag>
          )}
        </Space>
      </Space>

      <Space size="middle">
        {snapPoint && isDrawing && (
          <Tag color="green">
            吸附: {snapPoint.id === -1 ? "线段上" : `P${snapPoint.id}`}
          </Tag>
        )}
        <span className="text-gray-500">
          坐标: {mousePos.x.toFixed(2)}m, {mousePos.y.toFixed(2)}m
        </span>
      </Space>
    </div>
  );
};
