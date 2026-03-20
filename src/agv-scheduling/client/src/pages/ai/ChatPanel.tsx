/**
 * AI 智能交互面板
 *
 * 功能：输入文字 → 解析 → 预览任务 → 确认下发（通过 WebSocket 真实下发）
 */
import { useState, useRef, useEffect, useMemo, useCallback } from "react";
import {
  Input,
  Button,
  Card,
  Tag,
  Steps,
  Space,
  App,
  Empty,
  Spin,
  Typography,
  Divider,
  Select,
} from "antd";
import {
  SendOutlined,
  CheckCircleOutlined,
  CloseCircleOutlined,
  RobotOutlined,
  UserOutlined,
  ThunderboltOutlined,
  AudioOutlined,
  LoadingOutlined,
} from "@ant-design/icons";
import { servers, localStorageKey, agvKey } from "@/config";
import { randomInRange } from "@/lib";
import {
  agvForwardTask,
  agvPickAndDropTask,
  agvChargeTask,
  findClosestPoint,
  findShortestPath,
} from "@/lib/map-editor";
import useWebSocket from "@/hooks/useWebSocket";
import type {
  AGVStatus,
  LineRoute,
  ArcRoute,
  Point,
} from "@/types/map-editor";
import type { Building, Floor } from "@/types/building";

const { Text, Title } = Typography;
const { TextArea } = Input;

// ========== 网络请求（原生 fetch）==========

async function apiPost(path: string, body: any) {
  const token = localStorage.getItem(localStorageKey.token);
  const res = await fetch(`${servers.apiServer}${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json", Authorization: token || "" },
    body: JSON.stringify(body),
  });
  return res.json();
}

async function apiGet(path: string) {
  const token = localStorage.getItem(localStorageKey.token);
  const res = await fetch(`${servers.apiServer}${path}`, {
    headers: { Authorization: token || "" },
  });
  return res.json();
}

// ========== 类型 ==========

interface SubTaskPreview {
  step: number;
  action: string;
  target: string;
  offset1: number;
}

interface ParsedTask {
  taskType: "transport" | "move" | "charge" | "control";
  controlAction?: string;
  from?: string;
  to?: string;
  needPick?: boolean;
  needDrop?: boolean;
  confidence: number;
  rawText: string;
  description: string;
  subtaskPreview: SubTaskPreview[];
}

/** 路径摘要中每段路的详情 */
interface RouteSummaryItem {
  from: string;
  to: string;
  type: "直线" | "弯道" | "自旋";
  distance: number;
  limitSpeed?: number;
  doorId?: number;
  obstacleAreas?: number[];
  isObstacleDetour?: boolean;
}

/** 路径摘要（整条路径的汇总） */
interface RouteSummary {
  segments: RouteSummaryItem[];
  totalDistance: number;
  totalDoors: number;
  doorIds: number[];
  hasObstacleAreas: boolean;
}

interface ChatMessage {
  id: string;
  role: "user" | "assistant";
  content: string;
  timestamp: number;
  task?: ParsedTask;
  routeSummary?: RouteSummary;
  status?: "parsing" | "parsed" | "confirmed" | "failed";
  taskId?: string;
}

interface MapContext {
  building: Building;
  floor: Floor;
  points: Point[];
  lines: LineRoute[];
  paths: ArcRoute[];
  sizeRatio: number;
}

// ========== 常量 ==========

const TASK_TYPE_CONFIG: Record<string, { color: string; label: string }> = {
  transport: { color: "blue", label: "搬运任务" },
  move: { color: "green", label: "移动任务" },
  charge: { color: "orange", label: "充电任务" },
  control: { color: "red", label: "控制指令" },
};

const OFFSET_LABELS: Record<number, string> = {
  1: "直线行驶",
  3: "举升放货",
  4: "充电对接",
  9: "取货",
  99: "任务结束",
};

const QUICK_COMMANDS = [
  "把A货架的货送到B工位",
  "去1号点取货送到3号点",
  "从5号搬到8号",
  "去充电",
  "暂停",
];

const kChargeId = agvKey.chargePointId;

// ========== 辅助函数 ==========

/** Q006 → "6", P100 → "100" */
function parsePointId(qpId: string): string {
  const match = qpId.match(/^[QP]0*(\d+)$/i);
  return match ? match[1] : qpId;
}

/** 从寻路结果中提取路径摘要（门、避障、限速等） */
function buildRouteSummary(edges: (LineRoute | ArcRoute)[]): RouteSummary {
  const segments: RouteSummaryItem[] = [];
  const doorIds = new Set<number>();
  let totalDistance = 0;
  let hasObstacleAreas = false;

  for (const edge of edges) {
    const startId = edge.start.id ?? "?";
    const endId = edge.end.id ?? "?";
    const isArc = "center" in edge && !!edge.center;
    const routeSettings = edge.settings?.[`${startId}-${endId}`];

    if (routeSettings?.doorId && routeSettings.doorId > 0) {
      doorIds.add(routeSettings.doorId);
    }
    if (routeSettings?.obstacleAreas && routeSettings.obstacleAreas.length > 0) {
      hasObstacleAreas = true;
    }

    totalDistance += edge.distance;
    segments.push({
      from: `P${startId}`,
      to: `P${endId}`,
      type: isArc ? "弯道" : "直线",
      distance: Math.round(edge.distance),
      limitSpeed: routeSettings?.limitSpeed,
      doorId: routeSettings?.doorId,
      obstacleAreas: routeSettings?.obstacleAreas,
      isObstacleDetour: routeSettings?.isObstacleDetour,
    });
  }

  return {
    segments,
    totalDistance: Math.round(totalDistance),
    totalDoors: doorIds.size,
    doorIds: [...doorIds],
    hasObstacleAreas,
  };
}

/** 根据解析结果和地图数据，计算完整路径摘要 */
function computeRouteSummary(
  task: ParsedTask,
  scaledRoutes: (LineRoute | ArcRoute)[],
  mapCtx: MapContext,
  agvs: AGVStatus[],
  selectedAgvId?: string,
): RouteSummary | undefined {
  if (task.taskType === "control") return undefined;

  const agvStatus = agvs.find((a) => a.id === selectedAgvId && a.isOnline);
  if (!agvStatus) return undefined;

  const { building, floor, points, sizeRatio } = mapCtx;
  const offsetX = building.origin.x + floor.agv_origin.x;
  const offsetY = building.origin.y + floor.agv_origin.y;

  // 找到 AGV 最近的路径点
  const { closestPoint } = findClosestPoint({
    points: points.filter((e) => !e.isPickOrDrop),
    position: {
      x: (offsetX + agvStatus.position.y) * sizeRatio,
      y: (offsetY + agvStatus.position.x) * sizeRatio,
    },
    minDistance: Infinity,
  });
  if (!closestPoint) return undefined;

  const graphEdges = [...scaledRoutes];
  const startId = closestPoint.id!;

  // 收集所有路段
  let allEdges: (LineRoute | ArcRoute)[] = [];

  if (task.taskType === "transport" && task.from && task.to) {
    const pickId = parsePointId(task.from);
    const endId = parsePointId(task.to);
    const pickEdges = findShortestPath(graphEdges, startId, pickId);
    const dropEdges = findShortestPath(graphEdges, pickId, endId);
    if (pickEdges) allEdges.push(...pickEdges);
    if (dropEdges) allEdges.push(...dropEdges);
  } else if (task.taskType === "move" && task.to) {
    const endId = parsePointId(task.to);
    const edges = findShortestPath(graphEdges, startId, endId);
    if (edges) allEdges.push(...edges);
  } else if (task.taskType === "charge") {
    const edges = findShortestPath(graphEdges, startId, kChargeId);
    if (edges) allEdges.push(...edges);
  }

  if (allEdges.length === 0) return undefined;
  return buildRouteSummary(allEdges);
}

/** 加载 SVG 图片获取宽度（用于计算 sizeRatio） */
function loadSvgWidth(bd: number, fl: number): Promise<number> {
  return new Promise((resolve) => {
    const img = new Image();
    img.onload = () => resolve(img.naturalWidth);
    img.onerror = () => resolve(0);
    img.src = `/assets/svg/${bd}-${fl}.svg`;
  });
}

/** 将 webm/ogg 等浏览器录音 Blob 转为 16kHz 单声道 WAV Blob */
async function blobToWav(blob: Blob): Promise<Blob> {
  const arrayBuffer = await blob.arrayBuffer();
  const audioCtx = new AudioContext({ sampleRate: 16000 });
  const decoded = await audioCtx.decodeAudioData(arrayBuffer);
  await audioCtx.close();

  // 取单声道数据
  const samples = decoded.getChannelData(0);
  const pcm = new Int16Array(samples.length);
  for (let i = 0; i < samples.length; i++) {
    const s = Math.max(-1, Math.min(1, samples[i]));
    pcm[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
  }

  // 构造 WAV 文件
  const wavBuffer = new ArrayBuffer(44 + pcm.byteLength);
  const view = new DataView(wavBuffer);
  const writeStr = (offset: number, str: string) => {
    for (let i = 0; i < str.length; i++) view.setUint8(offset + i, str.charCodeAt(i));
  };
  writeStr(0, "RIFF");
  view.setUint32(4, 36 + pcm.byteLength, true);
  writeStr(8, "WAVE");
  writeStr(12, "fmt ");
  view.setUint32(16, 16, true);         // chunk size
  view.setUint16(20, 1, true);          // PCM
  view.setUint16(22, 1, true);          // mono
  view.setUint32(24, 16000, true);      // sample rate
  view.setUint32(28, 16000 * 2, true);  // byte rate
  view.setUint16(32, 2, true);          // block align
  view.setUint16(34, 16, true);         // bits per sample
  writeStr(36, "data");
  view.setUint32(40, pcm.byteLength, true);
  new Int16Array(wavBuffer, 44).set(pcm);

  return new Blob([wavBuffer], { type: "audio/wav" });
}

// ========== 主组件 ==========

export default function ChatPanel() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputText, setInputText] = useState("");
  const [loading, setLoading] = useState(false);
  const scrollRef = useRef<HTMLDivElement>(null);
  const { notification } = App.useApp();

  // AGV 状态与选择
  const [agvs, setAgvs] = useState<AGVStatus[]>([]);
  const [selectedAgvId, setSelectedAgvId] = useState<string>();

  // 地图上下文（用于构建真实 SubTask）
  const [mapCtx, setMapCtx] = useState<MapContext | null>(null);

  // 语音录入状态
  const [recording, setRecording] = useState(false);
  const [voiceLoading, setVoiceLoading] = useState(false);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const audioChunksRef = useRef<Blob[]>([]);

  // WebSocket 连接
  const { sendMessage: wsSend } = useWebSocket(servers.websocketServer, {
    onMessage(msg) {
      if (msg.type === "update") {
        setAgvs(msg.data);
        // 自动选择第一台在线的 AGV
        if (!selectedAgvId) {
          const online = msg.data.find((a: AGVStatus) => a.isOnline);
          if (online) setSelectedAgvId(online.id);
        }
      }
    },
  });

  // 加载地图数据
  useEffect(() => {
    (async () => {
      try {
        const building = localStorage.getItem(localStorageKey.building) ?? "6";
        const floor = localStorage.getItem(localStorageKey.floor) ?? "2";
        const json = await apiGet(
          `/api/v1/map/find?building=${building}&floor=${floor}`
        );
        if (json?.status !== 200 || !json.data) return;

        const { building: bd, floor: fl } = json.data as {
          building: Building;
          floor: Floor;
        };
        const points: Point[] = fl.points ?? [];
        const lines: LineRoute[] = fl.lines ?? [];
        const paths: ArcRoute[] = fl.paths ?? [];

        // 加载 SVG 获取 sizeRatio
        const svgWidth = await loadSvgWidth(bd.no, fl.no);
        const sizeRatio = svgWidth > 0 ? svgWidth / bd.width : 1;

        setMapCtx({ building: bd, floor: fl, points, lines, paths, sizeRatio });
      } catch (err) {
        console.error("加载地图数据失败:", err);
      }
    })();
  }, []);

  // 计算坐标转换后的路线（像素坐标 → AGV 真实坐标）
  const scaledRoutes = useMemo(() => {
    if (!mapCtx) return [];
    const { building, floor, lines, paths, sizeRatio } = mapCtx;
    const offsetX = building.origin.x + floor.agv_origin.x;
    const offsetY = building.origin.y + floor.agv_origin.y;

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
  }, [mapCtx]);

  // 自动滚动到底部
  useEffect(() => {
    scrollRef.current?.scrollTo({
      top: scrollRef.current.scrollHeight,
      behavior: "smooth",
    });
  }, [messages]);

  // 发送消息（文本解析）
  const handleSend = async (text?: string) => {
    const msg = (text || inputText).trim();
    if (!msg) return;

    const userMsg: ChatMessage = {
      id: `u_${Date.now()}`,
      role: "user",
      content: msg,
      timestamp: Date.now(),
    };

    const assistantMsg: ChatMessage = {
      id: `a_${Date.now()}`,
      role: "assistant",
      content: "",
      timestamp: Date.now(),
      status: "parsing",
    };

    setMessages((prev) => [...prev, userMsg, assistantMsg]);
    setInputText("");
    setLoading(true);

    try {
      const data = await apiPost("/api/v1/ai/parse", { text: msg });

      // 解析成功后尝试计算路径摘要
      let routeSummary: RouteSummary | undefined;
      if (data?.success && data.task && mapCtx && scaledRoutes.length > 0) {
        try {
          routeSummary = computeRouteSummary(data.task, scaledRoutes, mapCtx, agvs, selectedAgvId);
        } catch {
          // 路径摘要计算失败不影响主流程
        }
      }

      setMessages((prev) =>
        prev.map((m) =>
          m.id === assistantMsg.id
            ? {
                ...m,
                content: data?.success
                  ? data.task?.description ?? "解析成功"
                  : data?.error || "解析失败",
                task: data?.success ? data.task : undefined,
                routeSummary,
                status: data?.success ? "parsed" : "failed",
              }
            : m
        )
      );
    } catch {
      setMessages((prev) =>
        prev.map((m) =>
          m.id === assistantMsg.id
            ? { ...m, content: "网络错误，请检查后端是否启动", status: "failed" }
            : m
        )
      );
    } finally {
      setLoading(false);
    }
  };

  // 确认下发（通过 WebSocket 真实派发任务到 AGV）
  const handleConfirm = useCallback(
    (msgId: string, task: ParsedTask) => {
      const agvStatus = agvs.find((a) => a.id === selectedAgvId && a.isOnline);
      if (!agvStatus) {
        notification.error({ message: "请先选择在线的 AGV" });
        return;
      }

      // 控制指令：直接发 WebSocket
      if (task.taskType === "control") {
        if (task.controlAction === "pause" || task.controlAction === "cancel" || task.controlAction === "estop") {
          wsSend({ type: "stop", data: { agvId: agvStatus.id } });
        }
        setMessages((prev) =>
          prev.map((m) =>
            m.id === msgId ? { ...m, status: "confirmed", taskId: `CTRL_${Date.now()}` } : m
          )
        );
        notification.success({ message: "控制指令已发送" });
        return;
      }

      // 需要地图数据才能构建完整 SubTask
      if (!mapCtx || scaledRoutes.length === 0) {
        notification.error({ message: "地图数据未加载，无法下发任务" });
        return;
      }

      const { building, floor, points, sizeRatio } = mapCtx;
      const offsetX = building.origin.x + floor.agv_origin.x;
      const offsetY = building.origin.y + floor.agv_origin.y;

      // 查询 AGV 最近的路径点（P 点）
      const { closestPoint, distance } = findClosestPoint({
        points: points.filter((e) => !e.isPickOrDrop),
        position: {
          x: (offsetX + agvStatus.position.y) * sizeRatio,
          y: (offsetY + agvStatus.position.x) * sizeRatio,
        },
        minDistance: Infinity,
      });

      if (!closestPoint) {
        notification.error({ message: "无法找到 AGV 附近的坐标点" });
        return;
      }

      const scaledDistance = distance / sizeRatio;
      if (scaledDistance > 3000) {
        notification.error({ message: "AGV 距离坐标点太远，请手动移动 AGV 到附近" });
        return;
      }

      // 构建图的边集（添加 AGV 当前位置的虚拟起点）
      const graphEdges = [...scaledRoutes];
      let startId = randomInRange(20000, 30000).toString();
      if (scaledDistance > 500) {
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
        startId = closestPoint.id!;
      }

      try {
        let wsType: string;
        let tasks: import("@/types/map-editor").SubTask[] = [];

        if (task.taskType === "transport" && task.from && task.to) {
          // ===== 搬运任务 =====
          wsType = "pickDrop";
          const pickId = parsePointId(task.from);
          const endId = parsePointId(task.to);

          // 寻路：起点 → 取货点
          const pickEdges = findShortestPath(graphEdges, startId, pickId);
          if (!pickEdges || pickEdges.length === 0) {
            notification.error({ message: `无法找到到取货点 ${task.from} 的路径` });
            return;
          }
          // 寻路：取货点 → 放货点
          const dropEdges = findShortestPath(graphEdges, pickId, endId);
          if (!dropEdges || dropEdges.length < 2) {
            notification.error({ message: `无法找到到放货点 ${task.to} 的路径` });
            return;
          }

          // 取货阶段：前进到取货路段起点 + 倒车取货
          let pickStartAngle = agvStatus.angle;
          const pickEdge = pickEdges.pop()!;
          if (pickEdges.length > 0) {
            const ft = agvForwardTask(agvStatus.angle, pickEdges);
            tasks.push(...ft.tasks);
            pickStartAngle = ft.finalAngle;
          }
          const pt = agvPickAndDropTask(pickStartAngle, pickEdge as LineRoute, true);
          tasks.push(...pt.tasks);

          // 放货阶段：前进到放货路段起点 + 倒车放货
          const dropEdge = dropEdges.pop()!;
          const ft = agvForwardTask(pt.finalAngle, dropEdges);
          tasks.push(...ft.tasks);
          const dt = agvPickAndDropTask(ft.finalAngle, dropEdge as LineRoute, false);
          tasks.push(...dt.tasks);

          // 放货后返回前一个点
          const ft2 = agvForwardTask(dt.finalAngle, [
            { ...dropEdge, start: dropEdge.end, end: dropEdge.start },
          ]);
          tasks.push(...ft2.tasks);

        } else if (task.taskType === "move" && task.to) {
          // ===== 移动任务 =====
          wsType = "move";
          const endId = parsePointId(task.to);
          const edges = findShortestPath(graphEdges, startId, endId);
          if (!edges || edges.length === 0) {
            notification.error({ message: `无法找到到 ${task.to} 的路径` });
            return;
          }
          tasks = agvForwardTask(agvStatus.angle, edges).tasks;

        } else if (task.taskType === "charge") {
          // ===== 充电任务 =====
          wsType = "charge";
          const edges = findShortestPath(graphEdges, startId, kChargeId);
          if (!edges || edges.length === 0) {
            notification.error({ message: "无法找到到充电站的路径" });
            return;
          }
          tasks = agvChargeTask(kChargeId, { angle: agvStatus.angle, edges });

        } else {
          notification.error({ message: "不支持的任务类型或缺少必要参数" });
          return;
        }

        // 通过 WebSocket 下发
        wsSend({ type: wsType, data: { agvId: agvStatus.id, message: tasks } });

        setMessages((prev) =>
          prev.map((m) =>
            m.id === msgId
              ? { ...m, status: "confirmed", taskId: `T${Date.now()}` }
              : m
          )
        );
        notification.success({
          message: "任务已下发",
          description: `AGV ${agvStatus.id} | ${tasks.length} 个分步`,
        });
      } catch (err: any) {
        notification.error({
          message: "任务构建失败",
          description: err.message,
        });
      }
    },
    [agvs, selectedAgvId, mapCtx, scaledRoutes, wsSend, notification]
  );

  // ========== 语音录入 ==========

  /** 开始录音 */
  const startRecording = useCallback(async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const mediaRecorder = new MediaRecorder(stream, { mimeType: "audio/webm" });
      mediaRecorderRef.current = mediaRecorder;
      audioChunksRef.current = [];

      mediaRecorder.ondataavailable = (e) => {
        if (e.data.size > 0) audioChunksRef.current.push(e.data);
      };

      mediaRecorder.onstop = async () => {
        // 停止所有音轨
        stream.getTracks().forEach((t) => t.stop());

        const rawBlob = new Blob(audioChunksRef.current, { type: "audio/webm" });
        if (rawBlob.size < 1000) {
          notification.warning({ message: "录音太短，请重试" });
          return;
        }

        // 上传到后端 STT（先转 WAV 格式，whisper-server 只支持 wav）
        setVoiceLoading(true);
        try {
          const audioBlob = await blobToWav(rawBlob);
          const formData = new FormData();
          formData.append("audio", audioBlob, "recording.wav");
          formData.append("format", "wav");

          const token = localStorage.getItem(localStorageKey.token);
          const res = await fetch(`${servers.apiServer}/api/v1/ai/voice`, {
            method: "POST",
            headers: { Authorization: token || "" },
            body: formData,
          });
          const data = await res.json();

          if (data?.stt?.text) {
            // STT 成功，用转写文本走正常解析流程
            notification.info({ message: `语音识别: "${data.stt.text}"`, duration: 2 });
            handleSend(data.stt.text);
          } else {
            notification.error({ message: data?.error || "语音识别失败" });
          }
        } catch {
          notification.error({ message: "语音上传失败，请检查网络" });
        } finally {
          setVoiceLoading(false);
        }
      };

      mediaRecorder.start();
      setRecording(true);
    } catch (err: any) {
      notification.error({ message: "无法访问麦克风", description: err.message });
    }
  }, [notification, handleSend]);

  /** 停止录音 */
  const stopRecording = useCallback(() => {
    if (mediaRecorderRef.current && mediaRecorderRef.current.state === "recording") {
      mediaRecorderRef.current.stop();
    }
    setRecording(false);
  }, []);

  // 按回车发送
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // 在线 AGV 选项
  const agvOptions = useMemo(
    () =>
      agvs
        .filter((a) => a.isOnline)
        .map((a) => ({
          value: a.id,
          label: `AGV ${a.id} (电量: ${a.battery ?? 0}%)`,
        })),
    [agvs]
  );

  return (
    <div className="h-full flex flex-col bg-gray-50">
      {/* 头部 */}
      <div className="px-6 py-4 bg-white border-b flex items-center gap-3">
        <RobotOutlined className="text-2xl text-blue-500" />
        <div className="flex-1">
          <Title level={5} style={{ margin: 0 }}>
            智能任务助手
          </Title>
          <Text type="secondary" style={{ fontSize: 12 }}>
            输入自然语言描述，自动解析为任务指令
            <Tag color="orange" style={{ marginLeft: 8 }}>本地规则版</Tag>
          </Text>
        </div>
        {/* AGV 选择器 */}
        <Select
          style={{ width: 200 }}
          placeholder="选择 AGV"
          value={selectedAgvId}
          onChange={setSelectedAgvId}
          options={agvOptions}
          notFoundContent={
            <Text type="secondary" style={{ fontSize: 12 }}>
              无在线 AGV
            </Text>
          }
        />
        {/* 地图状态 */}
        <Tag color={mapCtx ? "green" : "red"}>
          {mapCtx ? "地图已加载" : "地图未加载"}
        </Tag>
      </div>

      {/* 消息列表 */}
      <div ref={scrollRef} className="flex-1 overflow-auto px-6 py-4 space-y-4">
        {messages.length === 0 && (
          <div className="flex flex-col items-center justify-center h-full text-gray-400">
            <RobotOutlined style={{ fontSize: 48, marginBottom: 16 }} />
            <Text type="secondary">输入指令开始，例如："把A货架的货送到B工位"</Text>
            <Divider>快捷指令</Divider>
            <Space wrap>
              {QUICK_COMMANDS.map((cmd) => (
                <Button
                  key={cmd}
                  size="small"
                  onClick={() => handleSend(cmd)}
                  icon={<ThunderboltOutlined />}
                >
                  {cmd}
                </Button>
              ))}
            </Space>
          </div>
        )}

        {messages.map((msg) => (
          <div
            key={msg.id}
            className={`flex ${msg.role === "user" ? "justify-end" : "justify-start"}`}
          >
            <div className={`max-w-lg ${msg.role === "user" ? "order-2" : ""}`}>
              {/* 头像 */}
              <div
                className={`flex items-center gap-2 mb-1 ${
                  msg.role === "user" ? "flex-row-reverse" : ""
                }`}
              >
                {msg.role === "user" ? (
                  <UserOutlined className="text-blue-500" />
                ) : (
                  <RobotOutlined className="text-green-500" />
                )}
                <Text type="secondary" style={{ fontSize: 12 }}>
                  {new Date(msg.timestamp).toLocaleTimeString()}
                </Text>
              </div>

              {/* 消息体 */}
              {msg.role === "user" ? (
                <div className="bg-blue-500 text-white px-4 py-2 rounded-lg rounded-tr-sm">
                  {msg.content}
                </div>
              ) : (
                <Card
                  size="small"
                  className="rounded-lg rounded-tl-sm"
                  styles={{ body: { padding: "12px 16px" } }}
                >
                  {msg.status === "parsing" ? (
                    <Spin size="small" tip="解析中..." />
                  ) : msg.status === "failed" ? (
                    <div>
                      <CloseCircleOutlined className="text-red-500 mr-2" />
                      <Text type="danger">{msg.content}</Text>
                    </div>
                  ) : (
                    <>
                      {/* 任务描述 */}
                      <div className="mb-2">
                        {msg.task && (
                          <Tag
                            color={TASK_TYPE_CONFIG[msg.task.taskType]?.color}
                          >
                            {TASK_TYPE_CONFIG[msg.task.taskType]?.label}
                          </Tag>
                        )}
                        <Text>{msg.content}</Text>
                        {msg.task && (
                          <Text
                            type="secondary"
                            style={{ fontSize: 12, marginLeft: 8 }}
                          >
                            置信度: {Math.round(msg.task.confidence * 100)}%
                          </Text>
                        )}
                      </div>

                      {/* SubTask 步骤预览 */}
                      {msg.task?.subtaskPreview &&
                        msg.task.subtaskPreview.length > 0 && (
                          <Steps
                            size="small"
                            direction="vertical"
                            current={
                              msg.status === "confirmed"
                                ? msg.task.subtaskPreview.length
                                : -1
                            }
                            items={msg.task.subtaskPreview.map((s) => ({
                              title: s.action,
                              description: (
                                <Text type="secondary" style={{ fontSize: 12 }}>
                                  目标: {s.target} | offset1={s.offset1} (
                                  {OFFSET_LABELS[s.offset1] || "未知"})
                                </Text>
                              ),
                            }))}
                          />
                        )}

                      {/* 路径摘要 */}
                      {msg.routeSummary && msg.routeSummary.segments.length > 0 && (
                        <div className="mt-2 mb-1 p-2 bg-gray-50 rounded text-xs">
                          <Text strong style={{ fontSize: 12 }}>
                            路径详情（{msg.routeSummary.segments.length} 段，
                            总距离 {(msg.routeSummary.totalDistance / 1000).toFixed(1)}m
                            {msg.routeSummary.totalDoors > 0 &&
                              `，${msg.routeSummary.totalDoors} 扇门`}
                            ）
                          </Text>
                          <div className="mt-1 space-y-0.5">
                            {msg.routeSummary.segments.map((seg, i) => (
                              <div key={i} className="flex items-center gap-1 text-gray-600">
                                <Text type="secondary" style={{ fontSize: 11 }}>
                                  {seg.from} → {seg.to}：{seg.type} {(seg.distance / 1000).toFixed(1)}m
                                  {seg.limitSpeed ? `，限速 ${seg.limitSpeed}` : ""}
                                </Text>
                                {seg.doorId && seg.doorId > 0 && (
                                  <Tag color="warning" style={{ fontSize: 10, lineHeight: "16px", padding: "0 4px", margin: 0 }}>
                                    {seg.doorId}号门
                                  </Tag>
                                )}
                                {seg.obstacleAreas && seg.obstacleAreas.length > 0 && (
                                  <Tag color="processing" style={{ fontSize: 10, lineHeight: "16px", padding: "0 4px", margin: 0 }}>
                                    避障{seg.obstacleAreas.join(",")}
                                  </Tag>
                                )}
                                {seg.isObstacleDetour && (
                                  <Tag color="purple" style={{ fontSize: 10, lineHeight: "16px", padding: "0 4px", margin: 0 }}>
                                    绕障
                                  </Tag>
                                )}
                              </div>
                            ))}
                          </div>
                          {msg.routeSummary.totalDoors === 0 &&
                            !msg.routeSummary.hasObstacleAreas && (
                            <Text type="secondary" style={{ fontSize: 11 }}>
                              此路径无卷帘门、无特殊避障区域
                            </Text>
                          )}
                        </div>
                      )}

                      {/* 操作按钮 */}
                      {msg.status === "parsed" && msg.task && (
                        <div className="mt-3 flex gap-2">
                          <Button
                            type="primary"
                            size="small"
                            icon={<CheckCircleOutlined />}
                            onClick={() => handleConfirm(msg.id, msg.task!)}
                          >
                            确认下发
                          </Button>
                          <Button
                            size="small"
                            onClick={() =>
                              setMessages((prev) =>
                                prev.map((m) =>
                                  m.id === msg.id
                                    ? { ...m, status: "failed", content: "已取消" }
                                    : m
                                )
                              )
                            }
                          >
                            取消
                          </Button>
                        </div>
                      )}

                      {/* 已确认状态 */}
                      {msg.status === "confirmed" && (
                        <Tag color="success" className="mt-2">
                          已下发 {msg.taskId}
                        </Tag>
                      )}
                    </>
                  )}
                </Card>
              )}
            </div>
          </div>
        ))}
      </div>

      {/* 快捷指令栏（有消息后） */}
      {messages.length > 0 && (
        <div className="px-6 py-2 border-t bg-white">
          <Space size={4} wrap>
            {QUICK_COMMANDS.map((cmd) => (
              <Button
                key={cmd}
                size="small"
                type="dashed"
                onClick={() => handleSend(cmd)}
                disabled={loading}
              >
                {cmd}
              </Button>
            ))}
          </Space>
        </div>
      )}

      {/* 输入框 */}
      <div className="px-6 py-4 bg-white border-t">
        <div className="flex gap-2">
          <TextArea
            value={inputText}
            onChange={(e) => setInputText(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder='输入任务指令，或按住麦克风说话'
            autoSize={{ minRows: 1, maxRows: 3 }}
            disabled={loading || recording}
            className="flex-1"
          />
          {/* 麦克风按钮：按下开始录音，松开停止 */}
          <Button
            type={recording ? "primary" : "default"}
            danger={recording}
            icon={voiceLoading ? <LoadingOutlined /> : <AudioOutlined />}
            onMouseDown={startRecording}
            onMouseUp={stopRecording}
            onMouseLeave={recording ? stopRecording : undefined}
            onTouchStart={startRecording}
            onTouchEnd={stopRecording}
            disabled={loading || voiceLoading}
            title="按住说话，松开识别"
          >
            {recording ? "松开发送" : voiceLoading ? "识别中" : "语音"}
          </Button>
          <Button
            type="primary"
            icon={<SendOutlined />}
            onClick={() => handleSend()}
            loading={loading}
            disabled={!inputText.trim()}
          >
            发送
          </Button>
        </div>
      </div>
    </div>
  );
}
