import { isMobile } from "react-device-detect";
import { useCallback, useEffect, useMemo, useState } from "react";
import {
  Minus,
  Plus,
  Grid,
  BrushCleaning,
  Spline,
  Send,
  SquareX,
  Space as SpaceIcon,
  MapPin,
  Save,
  MousePointer,
} from "lucide-react";
import {
  App,
  Button,
  Space,
  InputNumber,
  Checkbox,
  Tooltip,
  Divider,
  Select,
} from "antd";
import type {
  AGVStatus,
  Point,
  DrawMode,
  TaskMode,
  ConstraintOptions,
} from "@/types/map-editor";
import { localStorageKey } from "@/config";

interface ToolBarProps {
  drawMode: DrawMode;
  onSelectMode: (mode: DrawMode) => void;
  constraintOptions: ConstraintOptions;
  setConstraintOptions: (options: ConstraintOptions) => void;
  scale: number;
  setScale: (scale: number) => void;
  points: Point[];
  agvs: AGVStatus[];
  resetView: () => void;
  saveData: () => void;
  clearCanvas: () => void;
  sendCommand: (
    agvStatus: AGVStatus,
    type: TaskMode,
    pickId?: string,
    endId?: string,
    raiseHeight?: number
  ) => void;
  sendCancel: (agvStatus: AGVStatus) => void;
}

export const ToolBar = ({
  drawMode,
  onSelectMode,
  constraintOptions,
  setConstraintOptions,
  scale,
  setScale,
  points,
  agvs,
  resetView,
  saveData,
  clearCanvas,
  sendCommand,
  sendCancel,
}: ToolBarProps) => {
  // 使用 App.useApp() 获取 modal 和 message
  const { modal, message } = App.useApp();
  // 是否管理员权限
  const isAdmin = localStorage.getItem(localStorageKey.permission) === "admin";

  // 当前在线的agv状态
  const [agvStatus, setAgvStatus] = useState<AGVStatus>();
  // 任务模式
  const [taskMode, setTaskMode] = useState<TaskMode>("pickDrop");
  // 任务起止坐标点
  const [startId, setStartId] = useState<string>();
  const [endId, setEndId] = useState<string>();
  // 举升高度
  const [raiseHeight, setRaiseHeight] = useState(90);

  // 是否任务执行中 或 充电中
  const isRunning = useMemo(() => {
    const percent = agvStatus?.trCpltPercent ?? 0;
    return (percent > 0 && percent < 100) || agvStatus?.chargeMode === 1;
  }, [agvStatus]);

  // 排序后的坐标点
  const pointsSorted = useMemo(() => {
    return Array.from(points).sort(
      (a, b) => parseInt(a.id ?? "0") - parseInt(b.id ?? "0")
    );
  }, [points]);

  // 切换任务模式
  const onSwitchTaskMode = useCallback((mode: TaskMode) => {
    setTaskMode(mode);
    setStartId(undefined);
    setEndId(undefined);
  }, []);

  // 发送指令
  const onSendCommand = useCallback(() => {
    if (!agvStatus) {
      message.error("未找到在线的机器人");
      return;
    }

    if (taskMode === "pickDrop") {
      if (!startId) {
        message.error("请选择取货点");
        return;
      }
      if (!endId) {
        message.error("请选择卸货点");
        return;
      }
      if (startId === endId) {
        message.error("取货点和卸货点不能相同");
        return;
      }
    } else if (taskMode === "move") {
      if (!endId) {
        message.error("请选择终点");
        return;
      }
    }
    modal.confirm({
      title: "提醒",
      content:
        taskMode === "pickDrop"
          ? '请确保"取货点"已放置托盘，且"卸货点"已空出位置。'
          : "是否立即执行任务？",
      onOk: () => {
        const h = taskMode === "raise" ? raiseHeight : undefined;
        sendCommand(agvStatus, taskMode, startId, endId, h);
      },
    });
  }, [
    modal,
    message,
    agvStatus,
    taskMode,
    startId,
    endId,
    raiseHeight,
    sendCommand,
  ]);

  useEffect(() => {
    if (startId && !pointsSorted.some((p) => p.id == startId)) {
      setStartId(undefined);
    }
    if (endId && !pointsSorted.some((p) => p.id == endId)) {
      setEndId(undefined);
    }
  }, [startId, endId, pointsSorted]);

  useEffect(() => {
    const onlineAgvs = agvs.filter((e) => e.isOnline);
    if (onlineAgvs.length === 0) {
      setAgvStatus(undefined);
    } else {
      const ag = onlineAgvs[0];
      setAgvStatus(ag);
      if (ag.chargeMode == 1) setTaskMode("charge");
    }
  }, [agvs]);

  return (
    <div className="bg-white border-b p-3 flex items-center flex-wrap">
      {/* 添加编辑模式切换按钮，只有管理员才能修改 */}
      {!isMobile && isAdmin && (
        <>
          <Button
            onClick={() =>
              onSelectMode(drawMode === "none" ? "select" : "none")
            }
          >
            {drawMode === "none" ? "编辑" : "完成"}
          </Button>

          <Divider className="mx-4" type="vertical" />
        </>
      )}

      {/* 指令发送 */}
      {drawMode === "none" && (
        <Space size="middle">
          <Select
            popupMatchSelectWidth={100}
            options={[
              { label: "取货卸货", value: "pickDrop" },
              { label: "移动", value: "move" },
              { label: "举升", value: "raise" },
              { label: "充电", value: "charge" },
            ]}
            disabled={isRunning}
            value={taskMode}
            onChange={onSwitchTaskMode}
          />

          {taskMode === "pickDrop" && (
            <>
              <span className="text-sm text-gray-500">
                取货点：
                <Select
                  placeholder="请选择"
                  disabled={isRunning}
                  value={startId}
                  options={pointsSorted
                    .filter((p) => p.isPickOrDrop)
                    .map((p) => ({ label: `Q${p.id}`, value: p.id }))}
                  onChange={(v) => setStartId(v)}
                />
              </span>

              <span className="text-sm text-gray-500">
                卸货点：
                <Select
                  placeholder="请选择"
                  disabled={isRunning}
                  value={endId}
                  options={pointsSorted
                    .filter((p) => p.isPickOrDrop)
                    .map((p) => ({ label: `Q${p.id}`, value: p.id }))}
                  onChange={(v) => setEndId(v)}
                />
              </span>
            </>
          )}

          {taskMode === "raise" && (
            <span className="text-sm text-gray-500">
              高度：
              <InputNumber
                min={90}
                max={2800}
                suffix="mm"
                disabled={isRunning}
                defaultValue={raiseHeight}
                onChange={(v) => v && setRaiseHeight(v)}
              />
            </span>
          )}

          {taskMode === "move" && (
            <span className="text-sm text-gray-500">
              终点：
              <Select
                placeholder="请选择"
                disabled={isRunning}
                value={endId}
                options={pointsSorted
                  .filter((p) => !p.isPickOrDrop)
                  .map((p) => ({ label: `P${p.id}`, value: p.id }))}
                onChange={(v) => setEndId(v)}
              />
            </span>
          )}

          <Space>
            <Tooltip title={taskMode === "charge" ? "开始充电" : "执行任务"}>
              <Button
                danger
                type="text"
                icon={<Send size={16} />}
                disabled={isRunning}
                onClick={onSendCommand}
              />
            </Tooltip>

            {isRunning && (
              <Tooltip title={taskMode === "charge" ? "停止充电" : "取消任务"}>
                <Button
                  danger
                  type="text"
                  icon={<SquareX size={16} />}
                  onClick={() => agvStatus && sendCancel(agvStatus)}
                />
              </Tooltip>
            )}
          </Space>
        </Space>
      )}

      {/* 绘制模式 */}
      {drawMode !== "none" && (
        <Space size="middle">
          <Tooltip title="选择">
            <Button
              type={drawMode === "select" ? "primary" : "default"}
              icon={<MousePointer size={16} />}
              onClick={() => onSelectMode("select")}
            />
          </Tooltip>

          <Tooltip title="取放点">
            <Button
              type={drawMode === "point" ? "primary" : "default"}
              icon={<MapPin size={16} />}
              onClick={() => onSelectMode("point")}
            />
          </Tooltip>

          <Tooltip title="绘制直线">
            <Button
              type={drawMode === "straight" ? "primary" : "default"}
              icon={<SpaceIcon size={16} />}
              onClick={() => onSelectMode("straight")}
            />
          </Tooltip>

          <Tooltip title="绘制弧线">
            <Button
              className="mr-2"
              type={drawMode === "arc" ? "primary" : "default"}
              icon={<Spline size={16} />}
              onClick={() => onSelectMode("arc")}
            />
          </Tooltip>

          {/* 直线的约束条件 */}
          {drawMode === "straight" && (
            <Space size="small">
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
        </Space>
      )}

      {/* 导出、清空 */}
      <Space className="ml-auto">
        {drawMode !== "none" && (
          <>
            <Tooltip title="保存数据">
              <Button icon={<Save size={16} />} onClick={saveData} />
            </Tooltip>

            <Tooltip title="清空画布">
              <Button
                danger
                type="dashed"
                icon={<BrushCleaning size={16} />}
                onClick={clearCanvas}
              />
            </Tooltip>
          </>
        )}
      </Space>

      {/* 视图控制 */}
      <Space className="px-4">
        <Button
          icon={<Minus size={16} />}
          onClick={() => setScale(scale * 0.9)}
        />

        <span className="text-sm font-mono min-w-20 text-center">
          {scale.toFixed(1)}x
        </span>

        <Button
          icon={<Plus size={16} />}
          onClick={() => setScale(scale * 1.1)}
        />
      </Space>

      {/* 重置视图 */}
      <Tooltip title="重置视图">
        <Button type="primary" icon={<Grid size={16} />} onClick={resetView} />
      </Tooltip>
    </div>
  );
};

export default ToolBar;
