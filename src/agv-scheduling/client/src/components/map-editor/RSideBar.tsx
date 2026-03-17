import { useMemo, useState } from "react";
import {
  Checkbox,
  Col,
  Collapse,
  Empty,
  Input,
  Row,
  Radio,
  Select,
  Steps,
  Switch,
  InputNumber,
} from "antd";
import type { CollapseProps, StepProps } from "antd/lib";
import type { Door } from "@/types/door";
import type {
  DrawMode,
  SubTask,
  Point,
  PointSettings,
  ArcRoute,
  LineRoute,
  RouteSettings,
} from "@/types/map-editor";

const { Search } = Input;

interface RSideBarProps {
  drawMode: DrawMode;
  tasks: SubTask[];
  taskProgress: number;
  points: Point[];
  routes: (LineRoute | ArcRoute)[];
  doors: Door[];
  onPointSettingsChanged: (point: Point) => void;
  onRouteSettingsChanged: (route: LineRoute | ArcRoute) => void;
}

const RSideBar: React.FC<RSideBarProps> = ({
  drawMode,
  tasks,
  taskProgress,
  points,
  routes,
  doors,
  onPointSettingsChanged,
  onRouteSettingsChanged,
}) => {
  // 任务分步
  const stepItems = useMemo(() => tasks.map((e) => getStepItem(e)), [tasks]);
  const currentStep = useMemo(
    () => Math.round((taskProgress / 100) * tasks.length),
    [tasks, taskProgress]
  );

  // 路段设置
  const [searchText, setSearchText] = useState("");
  const collapseItems = useMemo(() => {
    // 1.坐标点
    const items: CollapseProps["items"] = points.map((point) =>
      getPointCollapseItem({
        id: point.id!,
        pointSettings: point.settings ?? { obstacleAreas: [] },
        onChanged: (v) => onPointSettingsChanged({ ...point, settings: v }),
      })
    );

    // 2.路段: 直线和弧线的正反方向
    routes.forEach((route) => {
      const isArc = "center" in route && !!route.center;
      const { start, end, settings = {} } = route;
      items.push(
        // 1.正向路段
        getRouteCollapseItem({
          isArc,
          startId: start.id!,
          endId: end.id!,
          doors,
          settings,
          onChanged: (v) => onRouteSettingsChanged({ ...route, settings: v }),
        }),
        // 2.反向路段
        getRouteCollapseItem({
          isArc,
          startId: end.id!,
          endId: start.id!,
          doors,
          settings,
          onChanged: (v) => onRouteSettingsChanged({ ...route, settings: v }),
        })
      );
    });

    return items;
  }, [points, routes, doors, onPointSettingsChanged, onRouteSettingsChanged]);

  if (drawMode === "none") {
    return tasks.length === 0 ? (
      <div className="flex items-center justify-center h-full">
        <Empty description="暂无任务" />
      </div>
    ) : (
      <Steps
        direction="vertical"
        size="small"
        current={currentStep}
        items={stepItems}
      />
    );
  }

  return routes.length === 0 ? (
    <div className="flex items-center justify-center h-full">
      <Empty />
    </div>
  ) : (
    <>
      <Search
        className="mb-3"
        allowClear
        placeholder="搜索坐标点"
        value={searchText}
        onChange={(e) => setSearchText(e.target.value)}
      />
      <Collapse
        size="small"
        items={collapseItems?.filter((e) =>
          (e.key as string).includes(searchText)
        )}
      />
    </>
  );
};

// 任务分步
const getStepItem = (task: SubTask): StepProps => {
  switch (task.offset1) {
    case 1: // 直线
      return {
        title: task.offset3 === -1 ? "前进" : "后退",
        description: `起点: ${task.offset19}, 目标点: ${
          task.offset20
        }, 距离: ${((task.offset4 ?? 0) / 1000).toFixed(1)}m`,
      };

    case 2: // 弯道
      return {
        title: task.offset19 === task.offset20 ? "原地旋转" : "转弯",
        description:
          task.offset19 === task.offset20
            ? `目标角度: ${Math.round(task.offset3 ?? 0)}°`
            : `起点: ${task.offset19}, 目标点: ${task.offset20}, 距离: ${(
                (task.offset4 ?? 0) / 1000
              ).toFixed(1)}m`,
      };

    case 3: // 举升
      return {
        title: "举升",
        description: `货叉高度: ${task.offset2}mm`,
      };

    case 4: // 充电
      return {
        title: task.offset2 === 1 ? "充电" : "停止充电",
      };

    case 9: // 自调节
      return {
        title: "取货",
        description: `目标点: ${task.offset20}, 距离: ${(
          (task.offset4 ?? 0) / 1000
        ).toFixed(1)}m`,
      };

    default:
      return { title: "未知" };
  }
};

// 路段设置
const getRouteCollapseItem = (args: {
  isArc: boolean;
  startId: string;
  endId: string;
  doors: Door[];
  settings: { [key: string]: RouteSettings };
  onChanged: (settings: { [key: string]: RouteSettings }) => void;
}) => {
  const { isArc, startId, endId, doors, settings, onChanged } = args;
  const key = `${startId}-${endId}`;
  const routeSettings: RouteSettings = settings[key] ?? {
    isObstacleDetour: false, // 是否绕障
    obstacleAreas: [], // 避障区域
  };
  return {
    key,
    label: `${isArc ? "弧线" : "直线"}：${startId} - ${endId}`,
    children: (
      <>
        <h4 className="mb-1">开启绕障: </h4>
        <Switch
          value={routeSettings.isObstacleDetour}
          onChange={(v) =>
            onChanged({
              ...settings,
              [key]: { ...routeSettings, isObstacleDetour: v },
            })
          }
        />
        {!routeSettings.isObstacleDetour && (
          <>
            <h4 className="mb-1 mt-2">关闭避障: </h4>
            <Checkbox.Group<number>
              value={routeSettings.obstacleAreas}
              onChange={(v) =>
                onChanged({
                  ...settings,
                  [key]: { ...routeSettings, obstacleAreas: v },
                })
              }
            >
              <Row>
                <Col span={8}>
                  <Checkbox value={5}>5</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={6}>6</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={7}>7</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={1}>1</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={2}>2</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={3}>3</Checkbox>
                </Col>
                <Col span={8}>
                  <Checkbox value={0}>0</Checkbox>
                </Col>
                <Col span={8} offset={8}>
                  <Checkbox value={4}>4</Checkbox>
                </Col>
              </Row>
            </Checkbox.Group>
          </>
        )}
        <h4 className="mb-1 mt-2">限速: </h4>
        <InputNumber
          style={{ width: "100%" }}
          min={0}
          suffix="mm/s"
          placeholder="请输入"
          value={routeSettings.limitSpeed}
          onChange={(v) =>
            onChanged({
              ...settings,
              [key]: { ...routeSettings, limitSpeed: v || undefined },
            })
          }
        />
        <h4 className="mb-1 mt-2">卷帘门: </h4>
        <Select
          style={{ width: "100%" }}
          placeholder="请选择"
          options={[{ id: 0, ip: "无" }, ...doors].map((e) => ({
            label: e.ip,
            value: e.id,
          }))}
          value={routeSettings.doorId}
          onChange={(v) =>
            onChanged({
              ...settings,
              [key]: { ...routeSettings, doorId: v },
            })
          }
        />
      </>
    ),
  };
};

const getPointCollapseItem = (args: {
  id: string;
  pointSettings: PointSettings;
  onChanged: (settings: PointSettings) => void;
}) => {
  const { id, pointSettings, onChanged } = args;
  return {
    key: id,
    label: `坐标点：${id}`,
    children: (
      <>
        <h4 className="mb-1">关闭避障（自旋）: </h4>
        <Checkbox.Group<number>
          value={pointSettings.obstacleAreas}
          onChange={(v) => onChanged({ ...pointSettings, obstacleAreas: v })}
        >
          <Row>
            <Col span={8}>
              <Checkbox value={5}>5</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={6}>6</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={7}>7</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={1}>1</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={2}>2</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={3}>3</Checkbox>
            </Col>
            <Col span={8}>
              <Checkbox value={0}>0</Checkbox>
            </Col>
            <Col span={8} offset={8}>
              <Checkbox value={4}>4</Checkbox>
            </Col>
          </Row>
        </Checkbox.Group>
        <h4 className="mb-1 mt-2">自旋方向: </h4>
        <Radio.Group
          style={{ width: "100%" }}
          options={[
            { label: "无", value: 0 },
            { label: "往左", value: 1 },
            { label: "往右", value: 2 },
          ]}
          value={pointSettings.rotationDirection}
          onChange={(e) =>
            onChanged({ ...pointSettings, rotationDirection: e.target.value })
          }
        />
      </>
    ),
  };
};

export default RSideBar;
