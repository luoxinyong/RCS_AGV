import { App } from "antd";
import { useRef, useState, useCallback, useEffect, useMemo } from "react";
import { servers, localStorageKey } from "@/config";
import { mapApi, doorApi } from "@/network/api";
import { randomInRange } from "@/lib";
import {
  agvChargeTask,
  agvForwardTask,
  agvPickAndDropTask,
  agvRaiseTask,
  findClosestPoint,
  findShortestPath,
} from "@/lib/map-editor";
import ToolBar from "@/components/map-editor/ToolBar";
import StatusBar from "@/components/map-editor/StatusBar";
import HelpPanel from "@/components/map-editor/HelpPanel";
import RSideBar from "@/components/map-editor/RSideBar";
import useApi from "@/hooks/useApi";
import useCanvas from "@/hooks/map-editor/useCanvas";
import useWebSocket from "@/hooks/useWebSocket";
import type { Building, Floor } from "@/types/building";
import type { Door } from "@/types/door";
import type {
  AGVStatus,
  ArcRoute,
  LineRoute,
  SubTask,
  TaskMode,
} from "@/types/map-editor";

// FIXME: 充电点
const kChargeId = "15";

const MapEditor: React.FC = () => {
  const { message, notification } = App.useApp();
  const containerRef = useRef<HTMLDivElement>(null);
  const counting = useRef(0);
  const [agvs, setAgvs] = useState<AGVStatus[]>([]);
  const [tasks, setTasks] = useState<SubTask[]>([]);
  const [taskProgress, setTaskProgress] = useState(0);

  const { execute: mapExecute, data: mapData } = useApi<{
    building: Building;
    floor: Floor;
  }>();

  const { execute: doorExecute, data: doorData } = useApi<Door[]>();

  // websocket钩子
  const { sendMessage } = useWebSocket(servers.websocketServer, {
    onMessage(msg) {
      const { type, data } = msg;
      switch (type) {
        case "update":
          setAgvs(data);
          if (data.length > 0) {
            const percent = agvs[0]?.trCpltPercent || 0;
            if (percent === 0 || percent === 100) {
              if (counting.current < 3) {
                counting.current++;
              } else {
                counting.current = 0;
                // 3秒后清空任务列表
                setTasks([]);
                setTaskProgress(0);
              }
            } else {
              setTaskProgress(percent);
            }
          }
          break;

        case "error":
          notification.error({
            message: data.title,
            description: data.message,
            duration: -1,
          });
          break;

        case "info":
          notification.info({
            message: data.title,
            description: data.message,
          });
          break;

        default:
          notification.error({ message: "未知错误" });
          break;
      }
    },
  });

  // fabric canvas钩子
  const {
    sizeRatio,
    scale,
    setScale,
    drawMode,
    points,
    lines,
    paths,
    snapPoint,
    constraintOptions,
    setConstraintOptions,
    resetView,
    clearCanvas,
    switchDrawMode,
    onPointSettingsChanged,
    onRouteSettingsChanged,
  } = useCanvas({ divEle: containerRef.current, mapData, agvs });

  const scaledRoutes = useMemo(() => {
    if (!mapData) return [];

    const offsetX = mapData.building.origin.x + mapData.floor.agv_origin.x;
    const offsetY = mapData.building.origin.y + mapData.floor.agv_origin.y;
    return [...lines, ...paths].map((route: LineRoute | ArcRoute) => {
      const scaledStart = {
        ...route.start,
        x: route.start.y / sizeRatio - offsetY,
        y: route.start.x / sizeRatio - offsetX,
      };
      const scaledEnd = {
        ...route.end,
        x: route.end.y / sizeRatio - offsetY,
        y: route.end.x / sizeRatio - offsetX,
      };
      const scaledCenter =
        "center" in route
          ? {
              ...route.center,
              x: route.center.y / sizeRatio - offsetY,
              y: route.center.x / sizeRatio - offsetX,
            }
          : undefined;
      const scaledDistance = route.distance / sizeRatio;
      return {
        ...route,
        start: scaledStart,
        end: scaledEnd,
        center: scaledCenter,
        distance: scaledDistance,
      } as LineRoute | ArcRoute;
    });
  }, [mapData, lines, paths, sizeRatio]);

  // 保存地图数据
  const saveData = useCallback(async () => {
    if (mapData) {
      const result = await mapApi.editMap({
        building: mapData.building.no.toString(),
        floor: mapData.floor.no.toString(),
        points: JSON.stringify(points),
        lines: JSON.stringify(lines),
        paths: JSON.stringify(paths),
      });
      if (result.status === 200) {
        notification.success({ message: "地图数据保存成功!" });
      } else {
        notification.error({
          message: "保存失败",
          description: result.statusText,
        });
      }
    }
  }, [mapData, notification, points, lines, paths]);

  // 发送任务
  const sendWsMessage = useCallback(
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (type: string, data: any) => {
      if (data.message) {
        counting.current = 0;
        setTasks(data.message);
      }
      sendMessage({ type, data });
    },
    [sendMessage]
  );

  // 发送指令
  const sendCommand = useCallback(
    (
      agvStatus: AGVStatus,
      type: TaskMode,
      pickId?: string,
      endId?: string,
      raiseHeight?: number
    ) => {
      if (!mapData) return;

      // 举升任务: 原地操作任务，不需要寻找路径
      if (raiseHeight) {
        const rts = [agvRaiseTask(raiseHeight, agvStatus.position)];
        sendWsMessage(type, { agvId: agvStatus.id, message: rts });
        return;
      }

      const offsetX = mapData.building.origin.x + mapData.floor.agv_origin.x;
      const offsetY = mapData.building.origin.y + mapData.floor.agv_origin.y;
      // 查询当前agv最近的坐标点
      const { closestPoint, distance } = findClosestPoint({
        points: points.filter((e) => !e.isPickOrDrop),
        position: {
          x: (offsetX + agvStatus.position.y) * sizeRatio,
          y: (offsetY + agvStatus.position.x) * sizeRatio,
        },
        minDistance: Infinity,
      });
      if (!closestPoint) {
        message.error("无法找到附近的坐标点");
        return;
      }
      const scaledDistance = distance / sizeRatio;
      if (scaledDistance > 3000) {
        message.error("出于安全考虑，请手动将AGV移动到坐标点附近");
        return;
      }

      const graphEdges = [...scaledRoutes];
      let startId = randomInRange(20000, 30000).toString();
      if (scaledDistance > 500) {
        // 如果agv的当前位置距离最近的坐标点太远，则创建一个新起点
        const lineRoute: LineRoute = {
          start: { ...agvStatus.position, id: startId },
          end: {
            id: closestPoint.id,
            x: closestPoint.y / sizeRatio - offsetY,
            y: closestPoint.x / sizeRatio - offsetX,
          },
          distance: scaledDistance,
        };
        graphEdges.push(lineRoute);
      } else {
        // 将最近的坐标点作为起点
        startId = closestPoint.id!;
      }

      const tasks: SubTask[] = [];
      if (pickId && endId) {
        const pickEdges = findShortestPath(graphEdges, startId, pickId);
        if (!pickEdges || pickEdges.length === 0) {
          message.error("取货任务：无法找到有效路径！");
          return;
        }
        const dropEdges = findShortestPath(graphEdges, pickId, endId);
        if (!dropEdges || dropEdges.length < 2) {
          message.error("卸货任务：无法找到有效路径！");
          return;
        }

        // 取货任务: 1.前进任务开到最后一段"取货路段"的起点
        let pickStartAngle = agvStatus.angle;
        const pickEdge = pickEdges.pop()!;
        if (pickEdges.length > 0) {
          const ft = agvForwardTask(agvStatus.angle, pickEdges)!;
          tasks.push(...ft.tasks);
          pickStartAngle = ft.finalAngle;
        }
        // 2.在最后一段路倒车取货
        const pt = agvPickAndDropTask(pickStartAngle, pickEdge, true);
        tasks.push(...pt.tasks);

        // 卸货任务: 1.前进任务开到最后一段"卸货路段"的起点
        const dropEdge = dropEdges.pop()!;
        const ft = agvForwardTask(pt.finalAngle, dropEdges);
        tasks.push(...ft.tasks);
        // 2.在最后一段路倒车卸货
        const dt = agvPickAndDropTask(ft.finalAngle, dropEdge, false);
        tasks.push(...dt.tasks);
        // 3.卸货完成后，返回倒数第二个坐标点
        const ft2 = agvForwardTask(dt.finalAngle, [
          {
            ...dropEdge,
            start: dropEdge.end,
            end: dropEdge.start,
          },
        ]);
        tasks.push(...ft2.tasks);

        // const pickEdges2 = findShortestPath(
        //   graphEdges,
        //   dropEdge.start.id!,
        //   endId
        // )!;
        // const dropEdges2 = findShortestPath(graphEdges, endId, pickId)!;
        // const pickEdge2 = pickEdges2.pop()!;
        // if (pickEdges2.length > 0) {
        //   const ft = agvForwardTask(ft2.finalAngle, pickEdges2)!;
        //   tasks.push(...ft.tasks);
        //   pickStartAngle = ft.finalAngle;
        // }
        // // 2.在最后一段路倒车取货
        // const pt2 = agvPickAndDropTask(pickStartAngle, pickEdge2, true);
        // tasks.push(...pt2.tasks);

        // // 卸货任务: 1.前进任务开到最后一段"卸货路段"的起点
        // const dropEdge2 = dropEdges2.pop()!;
        // const ft11 = agvForwardTask(pt2.finalAngle, dropEdges2);
        // tasks.push(...ft11.tasks);
        // // 2.在最后一段路倒车卸货
        // const dt2 = agvPickAndDropTask(ft11.finalAngle, dropEdge2, false);
        // tasks.push(...dt2.tasks);
        // // 3.卸货完成后，返回倒数第二个坐标点
        // const ft22 = agvForwardTask(dt2.finalAngle, [
        //   {
        //     ...dropEdge2,
        //     start: dropEdge2.end,
        //     end: dropEdge2.start,
        //   },
        // ]);
        // tasks.push(...ft22.tasks);

        // for (let i = 0; i < 4; i++) {
        //   const pickEdges = findShortestPath(graphEdges, startId, pickId)!;
        //   const dropEdges = findShortestPath(graphEdges, pickId, endId)!;
        //   pickStartAngle = ft22.finalAngle;
        //   const pickEdge = pickEdges.pop()!;
        //   if (pickEdges.length > 0) {
        //     const ft = agvForwardTask(agvStatus.angle, pickEdges)!;
        //     tasks.push(...ft.tasks);
        //     pickStartAngle = ft.finalAngle;
        //   }
        //   // 2.在最后一段路倒车取货
        //   const pt = agvPickAndDropTask(pickStartAngle, pickEdge, true);
        //   tasks.push(...pt.tasks);

        //   // 卸货任务: 1.前进任务开到最后一段"卸货路段"的起点
        //   const dropEdge = dropEdges.pop()!;
        //   const ft = agvForwardTask(pt.finalAngle, dropEdges);
        //   tasks.push(...ft.tasks);
        //   // 2.在最后一段路倒车卸货
        //   const dt = agvPickAndDropTask(ft.finalAngle, dropEdge, false);
        //   tasks.push(...dt.tasks);
        //   // 3.卸货完成后，返回倒数第二个坐标点
        //   const ft2 = agvForwardTask(dt.finalAngle, [
        //     {
        //       ...dropEdge,
        //       start: dropEdge.end,
        //       end: dropEdge.start,
        //     },
        //   ]);
        //   tasks.push(...ft2.tasks);

        //   const pickEdges2 = findShortestPath(
        //     graphEdges,
        //     dropEdge.start.id!,
        //     endId
        //   )!;
        //   const dropEdges2 = findShortestPath(graphEdges, endId, pickId)!;
        //   const pickEdge2 = pickEdges2.pop()!;
        //   if (pickEdges2.length > 0) {
        //     const ft = agvForwardTask(ft2.finalAngle, pickEdges2)!;
        //     tasks.push(...ft.tasks);
        //     pickStartAngle = ft.finalAngle;
        //   }
        //   // 2.在最后一段路倒车取货
        //   const pt2 = agvPickAndDropTask(pickStartAngle, pickEdge2, true);
        //   tasks.push(...pt2.tasks);

        //   // 卸货任务: 1.前进任务开到最后一段"卸货路段"的起点
        //   const dropEdge2 = dropEdges2.pop()!;
        //   const ft11 = agvForwardTask(pt2.finalAngle, dropEdges2);
        //   tasks.push(...ft11.tasks);
        //   // 2.在最后一段路倒车卸货
        //   const dt2 = agvPickAndDropTask(ft11.finalAngle, dropEdge2, false);
        //   tasks.push(...dt2.tasks);
        //   // 3.卸货完成后，返回倒数第二个坐标点
        //   const ft222 = agvForwardTask(dt2.finalAngle, [
        //     {
        //       ...dropEdge2,
        //       start: dropEdge2.end,
        //       end: dropEdge2.start,
        //     },
        //   ]);
        //   tasks.push(...ft222.tasks);
        // }
      } else if (endId) {
        // 移动
        const edges = findShortestPath(graphEdges, startId, endId);
        if (!edges || edges.length === 0) {
          message.error("移动任务：无法找到有效路径！");
          return;
        }
        tasks.push(...agvForwardTask(agvStatus.angle, edges).tasks);

        // 测试来回
        // const { finalAngle: fa1, tasks: t1 } = agvForwardTask(
        //   agvStatus.angle,
        //   edges
        // );
        // tasks.push(...t1); // 正
        // const { finalAngle: fa2, tasks: t2 } = agvForwardTask(
        //   fa1,
        //   findShortestPath(graphEdges, endId, startId)
        // );
        // tasks.push(...t2); // 反
        // for (let i = 0; i < 4; i++) {
        //   tasks.push(...agvForwardTask(fa2, edges).tasks); // 正
        //   tasks.push(...t2); // 反
        // }
      } else {
        // TODO: 寻找充电点
        const edges = findShortestPath(graphEdges, startId, kChargeId);
        if (!edges || edges.length === 0) {
          message.error("充电任务：无法找到有效路径！");
          return;
        }
        tasks.push(
          ...agvChargeTask(kChargeId, { angle: agvStatus.angle, edges })
        );
      }

      sendWsMessage(type, { agvId: agvStatus.id, message: tasks });
    },
    [mapData, message, points, sizeRatio, scaledRoutes, sendWsMessage]
  );

  // 取消执行中的任务
  const sendCancel = useCallback(
    (agvStatus: AGVStatus) => {
      // 停止充电，这里需要延时调用，否则ros会报udp断开错误
      if (agvStatus.chargeMode === 1) {
        sendWsMessage("stop_charge", {
          agvId: agvStatus.id,
          message: agvChargeTask(kChargeId),
        });
        setTimeout(() => {
          sendWsMessage("stop", { agvId: agvStatus.id });
        }, 1000);
      } else {
        sendWsMessage("stop", { agvId: agvStatus.id });
      }
    },
    [sendWsMessage]
  );

  useEffect(() => {
    const building = localStorage.getItem(localStorageKey.building) ?? "6";
    const floor = localStorage.getItem(localStorageKey.floor) ?? "2";
    mapExecute(() => mapApi.getMap({ building, floor }));
  }, [mapExecute]);

  useEffect(() => {
    doorExecute(doorApi.getDoors);
  }, [doorExecute]);

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
        points={points}
        agvs={agvs}
        resetView={resetView}
        saveData={saveData}
        clearCanvas={clearCanvas}
        sendCommand={sendCommand}
        sendCancel={sendCancel}
      />

      {/* 状态栏 */}
      <StatusBar
        drawMode={drawMode}
        pointLength={points.length}
        lineLength={lines.length}
        pathLength={paths.length}
        snapPoint={snapPoint}
      />

      {/* 画布容器 */}
      <div className="flex-1 overflow-hidden bg-[#eff6ff] flex">
        <div ref={containerRef} className="flex-1 min-w-0 mr-3" />
        <div className="w-[17rem] p-4 bg-white overflow-auto">
          <RSideBar
            drawMode={drawMode}
            tasks={tasks}
            taskProgress={taskProgress}
            points={points}
            routes={scaledRoutes}
            doors={doorData ?? []}
            onPointSettingsChanged={onPointSettingsChanged}
            onRouteSettingsChanged={onRouteSettingsChanged}
          />
        </div>
      </div>

      {/* 帮助提示 */}
      <HelpPanel />
    </div>
  );
};

export default MapEditor;
