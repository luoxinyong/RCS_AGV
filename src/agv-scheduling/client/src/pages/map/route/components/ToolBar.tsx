import {
  Minus,
  Plus,
  Grid,
  BrushCleaning,
  Pointer,
  BoxSelect,
} from "lucide-react";
import { Button, Space, InputNumber, Checkbox, Tooltip, Divider } from "antd";
import type { DrawMode, ConstraintOptions } from "../types";

interface ToolBarProps {
  drawMode: DrawMode;
  onSelectMode: (mode: DrawMode) => void;
  constraintOptions: ConstraintOptions;
  setConstraintOptions: (options: ConstraintOptions) => void;
  scale: number;
  setScale: (scale: number) => void;
  getUnit: () => { unit: string; factor: number; step: number };
  resetView: () => void;
  clearCanvas: () => void;
}

export const ToolBar = ({
  drawMode,
  onSelectMode,
  constraintOptions,
  setConstraintOptions,
  scale,
  setScale,
  getUnit,
  resetView,
  clearCanvas,
}: ToolBarProps) => {
  return (
    <div className="bg-white border-b p-3 flex items-center gap-2 flex-wrap">
      {/* 绘制模式 */}
      <Space>
        <Tooltip title="选择模式">
          <Button
            type={drawMode === "select" ? "primary" : "default"}
            icon={<Pointer size={16} />}
            onClick={() => {
              onSelectMode(drawMode === "select" ? "none" : "select");
            }}
          />
        </Tooltip>

        <Tooltip title="框选模式">
          <Button
            type={drawMode === "boxSelect" ? "primary" : "default"}
            icon={<BoxSelect size={16} />}
            onClick={() => {
              onSelectMode(drawMode === "boxSelect" ? "none" : "boxSelect");
            }}
          />
        </Tooltip>

        <Divider type="vertical" />

        <Button
          type={drawMode === "straight" ? "primary" : "default"}
          onClick={() => {
            onSelectMode(drawMode === "straight" ? "none" : "straight");
          }}
        >
          直线
        </Button>

        <Button
          type={drawMode === "arc" ? "primary" : "default"}
          onClick={() => {
            onSelectMode(drawMode === "arc" ? "none" : "arc");
          }}
        >
          弧线
        </Button>
      </Space>

      {/* 约束模式 */}
      {drawMode === "straight" && (
        <Space className="ml-2 border-l pl-4">
          <Checkbox
            checked={constraintOptions.isOrthogonal}
            onChange={(e) =>
              setConstraintOptions({
                ...constraintOptions,
                isOrthogonal: e.target.checked,
              })
            }
          >
            正交
          </Checkbox>

          <Checkbox
            checked={constraintOptions.isFixedLength}
            onChange={(e) =>
              setConstraintOptions({
                ...constraintOptions,
                isFixedLength: e.target.checked,
              })
            }
          >
            固定长度
          </Checkbox>

          {constraintOptions.isFixedLength && (
            <>
              <InputNumber
                size="small"
                min={1}
                max={100000}
                value={constraintOptions.fixedLength}
                onChange={(value) =>
                  setConstraintOptions({
                    ...constraintOptions,
                    fixedLength: value ?? 0,
                  })
                }
              />
              <span className="text-gray-500 text-sm">mm</span>
            </>
          )}
        </Space>
      )}

      {/* 视图控制 */}
      <Space className="ml-auto mr-2">
        <Button
          icon={<Minus size={16} />}
          onClick={() => setScale(scale * 0.9)}
        />

        <span className="text-sm font-mono min-w-20 text-center">
          {getUnit().unit} ({scale.toFixed(1)}x)
        </span>

        <Button
          icon={<Plus size={16} />}
          onClick={() => setScale(scale * 1.1)}
        />
      </Space>

      {/* 重置视图、清空画布 */}
      <Space>
        <Tooltip title="重置视图">
          <Button
            type="primary"
            icon={<Grid size={16} />}
            onClick={resetView}
          />
        </Tooltip>

        <Tooltip title="清空画布">
          <Button
            danger
            icon={<BrushCleaning size={16} />}
            onClick={clearCanvas}
          />
        </Tooltip>
      </Space>
    </div>
  );
};
