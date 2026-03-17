import { useState } from "react";
import { Card, List, Typography } from "antd";

export const HelpPanel = () => {
  const { Text } = Typography;

  const [isOpen, setIsOpen] = useState(false);

  // 基本操作提示
  const helpItems = [
    { key: "left", label: "左键", description: "绘制点/线或选择元素" },
    { key: "middle", label: "中键拖拽", description: "移动视图" },
    { key: "wheel", label: "滚轮", description: "缩放" },
    { key: "esc", label: "Esc", description: "取消绘制/选择" },
    { key: "delete", label: "Delete", description: "删除选中的点/线" },
  ];

  return (
    <>
      {/* 帮助按钮 */}
      <button
        title="快捷键帮助"
        className="absolute bottom-4 right-4 w-8 h-8 bg-gray-800 bg-opacity-70 text-white rounded-full flex items-center justify-center hover:bg-opacity-90 shadow-lg"
        onClick={() => setIsOpen(!isOpen)}
      >
        ?
      </button>

      {/* 帮助面板 */}
      {isOpen && (
        <div className="absolute bottom-14 right-4 rounded-lg w-64 border border-gray-200">
          <Card size="small" title="操作提示">
            <List
              size="small"
              split={false}
              dataSource={helpItems}
              renderItem={(item) => (
                <List.Item style={{ padding: "2px 0", fontSize: "0.5rem" }}>
                  <Text strong>{item.label}:</Text>{" "}
                  <Text>{item.description}</Text>
                </List.Item>
              )}
            />
          </Card>
        </div>
      )}
    </>
  );
};
