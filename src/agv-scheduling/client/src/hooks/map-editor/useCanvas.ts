import { App } from "antd";
import { fabric } from "fabric";
import { isMobile } from "react-device-detect";
import { useRef, useState, useEffect, useCallback } from "react";
import {
  createAgv,
  createCircle,
  createLine,
  createPath,
} from "@/lib/map-editor/fabric";
import {
  applyStraightConstraints,
  findNextId,
  findClosestPoint,
} from "@/lib/map-editor";
import type { Building, Floor } from "@/types/building";
import type {
  Point,
  DrawMode,
  ConstraintOptions,
  AGVStatus,
  LineRoute,
  ArcRoute,
} from "@/types/map-editor";

interface UseFabricCanvasProps {
  divEle: HTMLDivElement | null;
  mapData: { building: Building; floor: Floor } | null;
  agvs: AGVStatus[];
}

const useFabricCanvas = ({ divEle, mapData, agvs }: UseFabricCanvasProps) => {
  // 使用 App.useApp() 获取 modal 和 message
  const { modal, message } = App.useApp();
  // 创建Fabric.js画布引用
  const [canvas, setCanvas] = useState<fabric.Canvas>();

  // 状态管理
  const sizeRatio = useRef(1);
  const isDragging = useRef(false);
  const scaleCenter = useRef<fabric.Point>();
  const drawingStartPoint = useRef<Point>();
  const drawingMiddlePoint = useRef<Point>();

  const [scale, setScale] = useState(1);
  const [drawMode, setDrawMode] = useState<DrawMode>("none");
  const [constraintOptions, setConstraintOptions] = useState<ConstraintOptions>(
    {
      isOrthogonal: false,
      isFixedLength: false,
      fixedLength: 2000, // 默认2米
    }
  );
  const [snapPoint, setSnapPoint] = useState<Point>();
  const [points, setPoints] = useState<Point[]>([]);
  const [lines, setLines] = useState<LineRoute[]>([]);
  const [paths, setPaths] = useState<ArcRoute[]>([]);

  const clearPreview = useCallback(() => {
    // 删除预览对象
    canvas?.remove(...canvas.getObjects().filter((e) => e.data?.isPreview));

    // 重置绘制状态
    drawingStartPoint.current = undefined;
    drawingMiddlePoint.current = undefined;
  }, [canvas]);

  const checkRouteAlreadyExists = useCallback(
    (startId?: string, endId?: string) => {
      if (!startId || !endId) {
        return false;
      }
      const isExists = [...lines, ...paths].some((e) => {
        const { start, end } = e;
        return (
          (start.id === startId && end.id === endId) ||
          (start.id === endId && end.id === startId)
        );
      });
      if (isExists) {
        message.warning({ content: "该路段已存在，无法重复添加！" });
      }
      return isExists;
    },
    [message, lines, paths]
  );

  const finishDrawingStraight = useCallback(
    (startPoint: Point, endPoint: Point) => {
      if (!canvas || checkRouteAlreadyExists(startPoint.id, endPoint.id))
        return;

      // 检查起点是否在现有点上，如果不在则添加
      if (!startPoint.id) {
        startPoint.id = findNextId(points);
        canvas.add(...createCircle({ center: startPoint }));
        setPoints((v) => [...v, startPoint]);
      }

      // 检查终点是否在现有点上，如果不在则添加终点
      const endPointConstraints = applyStraightConstraints({
        start: startPoint,
        end: endPoint,
        isOrthogonal: constraintOptions.isOrthogonal,
        fixedLength: constraintOptions.isFixedLength
          ? constraintOptions.fixedLength * sizeRatio.current
          : undefined,
      });
      let end = points.find(
        (e) => e.x === endPointConstraints.x && e.y === endPointConstraints.y
      );
      if (!end) {
        end = {
          ...endPointConstraints,
          id: findNextId([...points, startPoint]),
        };
        canvas.add(...createCircle({ center: end }));
        setPoints((v) => [...v, end!]);
      }

      // 添加直线
      const line = createLine({ start: startPoint, end });
      canvas.add(line);
      setLines((v) => [...v, line.data]);
      setSnapPoint(end); // 完成直线绘制，把当前点设为吸附点
    },
    [canvas, points, constraintOptions, checkRouteAlreadyExists]
  );

  const finishDrawingArc = useCallback(
    (startPoint: Point, middlePoint: Point, endPoint: Point) => {
      if (!canvas || checkRouteAlreadyExists(startPoint.id, endPoint.id))
        return;

      // 检查起点是否在现有点上，如果不在则添加
      if (!startPoint.id) {
        startPoint.id = findNextId(points);
        canvas.add(...createCircle({ center: startPoint }));
        setPoints((v) => [...v, startPoint]);
      }

      // 检查终点是否在现有点上，如果不在则添加终点
      if (!endPoint.id) {
        endPoint.id = findNextId([...points, startPoint]);
        canvas.add(...createCircle({ center: endPoint }));
        setPoints((v) => [...v, endPoint]);
      }

      // 生成弧线路径
      const path = createPath({
        start: startPoint,
        middle: middlePoint,
        end: endPoint,
      });
      if (path) {
        canvas.add(path);
        setPaths((v) => [...v, path.data]);
        setSnapPoint(endPoint); // 完成直线绘制，把当前点设为吸附点
      }
    },
    [canvas, points, checkRouteAlreadyExists]
  );

  const handleMouseWheel = useCallback(
    (e: fabric.IEvent<MouseEvent>) => {
      if (!canvas) return;

      e.e.preventDefault(); // 阻止默认滚动行为
      e.e.stopPropagation(); // 阻止事件冒泡

      const zoom = canvas.getZoom();
      const delta = (e.e as WheelEvent).deltaY;
      let newZoom = delta > 0 ? zoom * 0.9 : zoom * 1.1;

      // 限制缩放范围
      if (newZoom > 10) newZoom = 10;
      if (newZoom < 0.1) newZoom = 0.1;

      // 记录鼠标当前位置，将以此为中心进行缩放
      const point = canvas.getPointer(e.e);
      scaleCenter.current = new fabric.Point(point.x, point.y);

      // 更新缩放状态
      setScale(newZoom);
    },
    [canvas]
  );

  const handleMouseDown = useCallback(
    (e: fabric.IEvent<MouseEvent>) => {
      if (!canvas) return;

      // 使用吸附点或当前位置
      const touchPoint: Point = snapPoint ?? canvas.getPointer(e.e, false);

      switch (drawMode) {
        case "point":
          {
            // 添加取放点
            const center: Point = {
              ...touchPoint,
              id: findNextId(points).toString(),
              isPickOrDrop: true,
            };
            canvas.add(...createCircle({ center }));
            setPoints((v) => [...v, center]);
          }
          break;

        case "straight":
          if (!drawingStartPoint.current) {
            // 如果起点不在现有点上，则创建新的起点
            drawingStartPoint.current = touchPoint;
            if (!snapPoint) {
              const p = createCircle({ center: touchPoint, isPreview: true });
              canvas.add(...p);
            }
          } else {
            // 完成直线绘制
            finishDrawingStraight(drawingStartPoint.current, touchPoint);
            // 重置状态
            clearPreview();
          }
          break;

        case "arc":
          if (!drawingStartPoint.current) {
            // 设置起点
            drawingStartPoint.current = touchPoint;
            if (!snapPoint) {
              const p = createCircle({ center: touchPoint, isPreview: true });
              canvas.add(...p);
            }
          } else if (!drawingMiddlePoint.current) {
            // 设置中间点
            drawingMiddlePoint.current = touchPoint;
            if (!snapPoint) {
              const p = createCircle({ center: touchPoint, isPreview: true });
              canvas.add(...p);
            }
          } else {
            // 完成弧线绘制
            finishDrawingArc(
              drawingStartPoint.current,
              drawingMiddlePoint.current,
              touchPoint
            );
            // 重置状态
            clearPreview();
          }
          break;
      }
    },
    [
      canvas,
      drawMode,
      snapPoint,
      points,
      finishDrawingStraight,
      finishDrawingArc,
      clearPreview,
    ]
  );

  const handleMouseMove = useCallback(
    (e: fabric.IEvent<MouseEvent>) => {
      if (!canvas || isDragging.current) return;

      // 1.获取鼠标位置
      const point = canvas.getPointer(e.e, false);

      if (drawMode === "straight" || drawMode === "arc") {
        // 2.更新吸附点
        const { closestPoint } = findClosestPoint({
          points: points,
          position: point,
          minDistance: 10,
        });
        setSnapPoint(closestPoint);

        // 3.绘制预览
        if (drawingStartPoint.current) {
          // 删除旧的预览，保留坐标点
          const items = canvas
            .getObjects()
            .filter(
              (e) => e.data?.isPreview && (e.type == "line" || e.type == "path")
            );
          canvas.remove(...items);

          // 预览终点
          const endPoint: Point = closestPoint || point;
          if (drawingMiddlePoint.current) {
            // 绘制弧线预览
            const path = createPath({
              start: drawingStartPoint.current,
              middle: drawingMiddlePoint.current,
              end: endPoint,
              isPreview: true,
            });
            if (path) canvas.add(path);
          } else {
            // 绘制直线预览
            const line = createLine({
              start: drawingStartPoint.current,
              end:
                drawMode === "straight"
                  ? applyStraightConstraints({
                      start: drawingStartPoint.current,
                      end: endPoint,
                      isOrthogonal: constraintOptions.isOrthogonal,
                      fixedLength: constraintOptions.isFixedLength
                        ? constraintOptions.fixedLength * sizeRatio.current
                        : undefined,
                    })
                  : endPoint,
              isPreview: true,
            });
            canvas.add(line);
          }
        }
      }
    },
    [canvas, drawMode, points, constraintOptions]
  );

  // 切换绘制模式
  const switchDrawMode = useCallback(
    (mode: DrawMode) => {
      setDrawMode(mode);
      clearPreview();

      if (canvas) {
        canvas.discardActiveObject();
        canvas.selection = mode === "select";
        canvas.defaultCursor =
          mode === "point" || mode === "straight" || mode === "arc"
            ? "pointer"
            : "default";
        canvas
          .getObjects()
          .filter(
            (e) => e.type === "path" || e.type === "line" || e.type === "circle"
          )
          .forEach((e) => (e.evented = mode === "select"));
      }
    },
    [canvas, clearPreview]
  );

  // 清空画布
  const clearCanvas = useCallback(() => {
    modal.confirm({
      title: "提醒",
      content: "确定要清空画布吗？",
      onOk: () => {
        // 只留下地图和agv
        canvas?.remove(
          ...canvas
            .getObjects()
            .filter((e) => e.name !== "base_floor" && !e.data?.id)
        );
        // 重置: 点、直线、弧线
        setPoints([]);
        setLines([]);
        setPaths([]);
      },
    });
  }, [canvas, modal]);

  // 重置视图
  const resetView = useCallback(() => {
    setScale(1);
    canvas?.setViewportTransform([1, 0, 0, 1, 0, 0]);
  }, [canvas]);

  // 修改坐标点配置
  const onPointSettingsChanged = useCallback(
    (point: Point) => {
      const { id, settings } = point;
      setPoints((v) =>
        v.map((e) => {
          if (e.id === id) {
            e.settings = settings;
          }
          return e;
        })
      );
      if (lines.some((e) => e.start.id == id || e.end.id == id)) {
        setLines((v) =>
          v.map((e) => {
            if (e.start.id === id) {
              e.start.settings = settings;
            } else if (e.end.id === id) {
              e.end.settings = settings;
            }
            return e;
          })
        );
      }
      if (paths.some((e) => e.start.id == id || e.end.id == id)) {
        setPaths((v) =>
          v.map((e) => {
            if (e.start.id === id) {
              e.start.settings = settings;
            } else if (e.end.id === id) {
              e.end.settings = settings;
            }
            return e;
          })
        );
      }
    },
    [lines, paths]
  );

  // 修改路径配置
  const onRouteSettingsChanged = useCallback((route: LineRoute | ArcRoute) => {
    const { start, end, settings } = route;
    if ("center" in route && route.center) {
      // 圆弧路径
      setPaths((v) =>
        v.map((e) => {
          if (e.start.id === start.id && e.end.id === end.id) {
            e.settings = settings;
          }
          return e;
        })
      );
    } else {
      // 直线路径
      setLines((v) =>
        v.map((e) => {
          if (e.start.id === start.id && e.end.id === end.id) {
            e.settings = settings;
          }
          return e;
        })
      );
    }
  }, []);

  // 监听agv状态
  useEffect(() => {
    if (!canvas || !mapData) return;

    // 删除旧的AGV
    canvas.remove(...canvas.getObjects().filter((e) => e.data?.id));
    if (agvs.length === 0) return;

    // 创建新的AGV
    const offsetX = mapData.building.origin.x + mapData.floor.agv_origin.x;
    const offsetY = mapData.building.origin.y + mapData.floor.agv_origin.y;
    canvas.add(
      ...agvs
        .filter((e) => e.position)
        .map((status) =>
          createAgv({
            status,
            angleFixed: 180 - status.angle, // 逆时针旋转
            positionFixed: {
              x: (offsetX + status.position.y) * sizeRatio.current,
              y: (offsetY + status.position.x) * sizeRatio.current,
            },
          })
        )
    );
  }, [canvas, agvs, mapData]);

  // fabric事件监听
  useEffect(() => {
    canvas?.on("mouse:wheel", handleMouseWheel);
    canvas?.on("mouse:down", handleMouseDown);
    canvas?.on("mouse:move", handleMouseMove);

    return () => {
      canvas?.off("mouse:wheel");
      canvas?.off("mouse:down");
      canvas?.off("mouse:move");
    };
  }, [canvas, handleMouseWheel, handleMouseDown, handleMouseMove]);

  // 设置缩放级别
  useEffect(() => {
    if (canvas) {
      // 1. 如果正在拖拽/缩放中（移动端），则跳过 Effect 同步，避免与 Event 同步冲突导致抖动
      if (isDragging.current) {
        scaleCenter.current = undefined;
        return;
      }

      // 2. 如果缩放比例未发生明显变化，则跳过（避免滚轮缩放时的冗余计算）
      if (Math.abs(canvas.getZoom() - scale) < 0.00001) {
        scaleCenter.current = undefined;
        return;
      }

      const c = canvas.getCenter();
      canvas.zoomToPoint(
        scaleCenter.current ?? new fabric.Point(c.left, c.top),
        scale
      );
      scaleCenter.current = undefined;
    }
  }, [canvas, scale]);

  // 加载楼层平面图
  useEffect(() => {
    if (!canvas || !mapData) return;

    const { no: bd, width } = mapData.building;
    const { no: fl, points, lines, paths } = mapData.floor;

    // 加载地图。原来: fabric.loadSVGFromURL
    fabric.Image.fromURL(`/assets/svg/${bd}-${fl}.svg`, (group) => {
      canvas.clear(); // 清空画布
      // const group = new fabric.Group(result, {
      //   name: "base_floor",
      //   evented: false,
      //   objectCaching: false,
      //   strokeUniform: true,
      // });
      group.name = "base_floor";
      group.evented = false;
      group.objectCaching = false;
      group.strokeUniform = true;
      // group.rotate(1); // 旋转1度
      canvas.add(group);

      // 设置宽高比例
      sizeRatio.current = (group.width ?? 0) / width;

      // 1.添加坐标点
      if (points && points.length > 0) {
        const items = points.reduce((p, e) => {
          p.push(...createCircle({ center: e }));
          return p;
        }, [] as fabric.Object[]);
        canvas.add(...items);
        setPoints(points);
      }
      // 2.添加直线
      if (lines && lines.length > 0) {
        canvas.add(...lines.map((e) => createLine(e)));
        setLines(lines);
      }
      // 3.添加弧线
      if (paths && paths.length > 0) {
        canvas.add(...paths.map((e) => createPath(e)).filter((e) => !!e));
        setPaths(paths);
      }
    });
  }, [mapData, canvas]);

  // window事件监听
  useEffect(() => {
    if (!canvas) return;

    if (isMobile && divEle) {
      // 移动端：实现单指拖动 + 双指缩放
      let lastTouchPosition: { x: number; y: number } | undefined;
      let lastPinchDistance: number | undefined;

      const getDistance = (p1: Touch, p2: Touch) => {
        return Math.sqrt(
          Math.pow(p2.clientX - p1.clientX, 2) +
            Math.pow(p2.clientY - p1.clientY, 2)
        );
      };

      const handleTouchStart = (e: TouchEvent) => {
        if (e.touches.length === 1) {
          // 单指：开始拖动
          isDragging.current = true;
          const p = e.touches[0];
          lastTouchPosition = { x: p.clientX, y: p.clientY };
        } else if (e.touches.length === 2) {
          // 双指：记录初始距离
          lastPinchDistance = getDistance(e.touches[0], e.touches[1]);
        }
      };

      const handleTouchMove = (e: TouchEvent) => {
        e.preventDefault(); // 阻止默认滚动行为

        if (isDragging.current && e.touches.length === 1 && lastTouchPosition) {
          // 单指拖动
          const p = e.touches[0];
          const point = new fabric.Point(
            p.clientX - lastTouchPosition.x,
            p.clientY - lastTouchPosition.y
          );
          canvas.relativePan(point);
          lastTouchPosition = { x: p.clientX, y: p.clientY };
        } else if (e.touches.length === 2 && lastPinchDistance) {
          // 双指缩放
          const dist = getDistance(e.touches[0], e.touches[1]);
          const zoom = canvas.getZoom();
          const scaleFactor = dist / lastPinchDistance;
          let newZoom = zoom * scaleFactor;

          // 限制缩放范围
          if (newZoom > 10) newZoom = 10;
          if (newZoom < 0.1) newZoom = 0.1;

          // 计算双指中心点作为缩放中心
          const p1 = e.touches[0];
          const p2 = e.touches[1];
          const cx = (p1.clientX + p2.clientX) / 2;
          const cy = (p1.clientY + p2.clientY) / 2;

          // 获取相对于canvas的中心点坐标
          const centerPoint = canvas.getPointer({
            clientX: cx,
            clientY: cy,
          } as MouseEvent);
          const point = new fabric.Point(centerPoint.x, centerPoint.y);
          scaleCenter.current = point;

          // 立即应用缩放，防止抖动
          canvas.zoomToPoint(point, newZoom);

          // 更新缩放
          setScale(newZoom);
          lastPinchDistance = dist;
        }
      };

      const handleTouchEnd = (e: TouchEvent) => {
        if (e.touches.length === 0) {
          isDragging.current = false;
          lastTouchPosition = undefined;
          lastPinchDistance = undefined;
        } else if (e.touches.length === 1) {
          // 从双指变为单指时，重置拖动起始点，防止跳动
          const p = e.touches[0];
          lastTouchPosition = { x: p.clientX, y: p.clientY };
          lastPinchDistance = undefined;
          isDragging.current = true;
        }
      };

      divEle.addEventListener("touchstart", handleTouchStart, {
        passive: false,
      });
      divEle.addEventListener("touchmove", handleTouchMove, {
        passive: false,
      });
      window.addEventListener("touchend", handleTouchEnd);
      return () => {
        divEle.removeEventListener("touchstart", handleTouchStart);
        divEle.removeEventListener("touchmove", handleTouchMove);
        window.removeEventListener("touchend", handleTouchEnd);
      };
    }

    const handleMouseDownEl = (e: MouseEvent) => {
      if (e.button === 1) {
        // 开始拖动画布
        isDragging.current = true;
        canvas.defaultCursor = "grabbing";
      }
    };

    const handleMouseMoveEl = (e: MouseEvent) => {
      if (isDragging.current) {
        const point = new fabric.Point(e.movementX, e.movementY);
        canvas.relativePan(point);
      }
    };

    const handleMouseUpEl = (e: MouseEvent) => {
      if (e.button === 1) {
        // 结束拖动画布
        isDragging.current = false;
        canvas.defaultCursor =
          drawMode === "point" || drawMode === "straight" || drawMode === "arc"
            ? "pointer"
            : "default";
      }
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      // 清空预览对象
      if (e.key === "Escape") {
        clearPreview();
      }
      // 删除选中的对象
      if (e.key === "Delete" || e.key === "Backspace") {
        for (const obj of canvas.getActiveObjects()) {
          if (obj.type === "circle") {
            // 1.删除点
            const circleAndText = canvas
              .getObjects()
              .filter((e) => e.name === obj.name);
            canvas.remove(...circleAndText);
            setPoints((pts) => pts.filter((e) => obj.data?.center.id !== e.id));
          } else if (obj.type === "line") {
            // 2.删除直线
            canvas.remove(obj);
            const { start: s1, end: e1 } = obj.data;
            setLines((ls) =>
              ls.filter((route) => {
                const { start: s2, end: e2 } = route;
                return s1.id !== s2.id || e1.id !== e2.id;
              })
            );
          } else if (obj.type === "path") {
            // 3.删除弧线
            canvas.remove(obj);
            const { start: s1, end: e1 } = obj.data;
            setPaths((ps) =>
              ps.filter((route) => {
                const { start: s2, end: e2 } = route;
                return s1.id !== s2.id || e1.id !== e2.id;
              })
            );
          }
        }
        canvas.discardActiveObject();
      }
      // 撤销操作
      // if (e.ctrlKey && e.key === "z") {
      // }
    };

    const handleResize = () => {
      if (divEle) {
        const { width, height } = divEle.getBoundingClientRect();
        canvas.setWidth(width);
        canvas.setHeight(height);
        canvas.renderAll();
      }
    };

    window.addEventListener("mousedown", handleMouseDownEl);
    window.addEventListener("mousemove", handleMouseMoveEl);
    window.addEventListener("mouseup", handleMouseUpEl);
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("mousedown", handleMouseDownEl);
      window.removeEventListener("mousemove", handleMouseMoveEl);
      window.removeEventListener("mouseup", handleMouseUpEl);
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("resize", handleResize);
    };
  }, [divEle, canvas, drawMode, clearPreview]);

  // 初始化画布
  useEffect(() => {
    if (!divEle) return;

    // 获取容器尺寸
    const { width, height } = divEle.getBoundingClientRect();

    // 创建canvas元素
    const cavEle = document.createElement("canvas");
    cavEle.id = "fabric-canvas";
    cavEle.width = width;
    cavEle.height = height;
    divEle.innerHTML = "";
    divEle.appendChild(cavEle);

    // 初始化Fabric.js画布
    const cav = new fabric.Canvas(cavEle, {
      width,
      height,
      // skipTargetFind: true,
      // 关闭鼠标右击事件
      stopContextMenu: true,
      // 优化渲染质量
      enableRetinaScaling: true, // 启用Retina屏幕支持
      imageSmoothingEnabled: true, // 启用图像平滑
      // 选择相关
      selection: false,
      selectionColor: "rgba(24, 144, 255, 0.1)",
      selectionDashArray: [5, 5],
      selectionBorderColor: "#1890ff",
      selectionFullyContained: true,
    });
    setCanvas(cav);

    return () => {
      cav.dispose();
      setCanvas(undefined);
    };
  }, [divEle]);

  // 返回钩子的API
  return {
    sizeRatio: sizeRatio.current,
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
  };
};

export default useFabricCanvas;
