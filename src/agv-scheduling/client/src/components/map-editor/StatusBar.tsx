import { Tag, Space } from "antd";
import { useCallback } from "react";
import type { DrawMode, Point } from "@/types/map-editor";

interface StatusBarProps {
  drawMode: DrawMode;
  pointLength: number;
  lineLength: number;
  pathLength: number;
  snapPoint?: Point;
}

export const StatusBar = ({
  drawMode,
  pointLength,
  lineLength,
  pathLength,
  snapPoint,
}: StatusBarProps) => {
  // 获取当前模式对应的标签类型
  const getModeTagType = useCallback(() => {
    switch (drawMode) {
      case "select":
        return "blue";
      case "point":
        return "orange";
      case "straight":
        return "green";
      case "arc":
        return "purple";
      default:
        return "default";
    }
  }, [drawMode]);

  // 获取模式显示文本
  const getModeText = useCallback(() => {
    switch (drawMode) {
      case "select":
        return "选择";
      case "point":
        return "添加取放点";
      case "straight":
        return "绘制直线";
      case "arc":
        return "绘制弧线";
      default:
        return "查看";
    }
  }, [drawMode]);

  return (
    <div className="bg-white border-b px-4 py-2 text-sm flex items-center justify-between">
      <Space size="small">
        <span>模式:</span>
        <Tag color={getModeTagType()}>{getModeText()}</Tag>
      </Space>

      <Space size="middle">
        {drawMode != "none" && snapPoint && (
          <Tag color="green">
            吸附: {(snapPoint.isPickOrDrop ? "Q" : "P") + snapPoint.id}
          </Tag>
        )}
        <Space size="small">
          <span>点: {pointLength}</span>
          <span>直线: {lineLength}</span>
          <span>弧线: {pathLength}</span>
        </Space>
      </Space>
    </div>
  );
};

export default StatusBar;
