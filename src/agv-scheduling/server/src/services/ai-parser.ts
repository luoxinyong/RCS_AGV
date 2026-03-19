/**
 * AI 文本解析服务（本地规则版）
 *
 * 职责：将自然语言文本解析为结构化的任务 JSON
 * 设计原则：回内网后只替换这一个文件（改为调大模型API），其他代码不动
 *
 * 支持的指令模式：
 *   - "把A的货送到B"  → 取货+送货
 *   - "去X取货送到Y"  → 取货+送货
 *   - "从X搬到Y"      → 取货+送货
 *   - "去充电"         → 充电任务
 *   - "暂停" / "继续" / "取消" → 控制指令
 *   - "去X点"          → 移动任务
 */

// ========== 类型定义 ==========

export interface ParsedTask {
  /** 任务类型 */
  taskType: "transport" | "move" | "charge" | "control";
  /** 控制类型（仅 control 任务） */
  controlAction?: "pause" | "resume" | "cancel" | "estop";
  /** 起点名称或编号 */
  from?: string;
  /** 终点名称或编号 */
  to?: string;
  /** 是否需要取货 */
  needPick?: boolean;
  /** 是否需要放货 */
  needDrop?: boolean;
  /** 解析置信度 0~1 */
  confidence: number;
  /** 原始文本 */
  rawText: string;
  /** 解析说明（给前端展示） */
  description: string;
  /** 生成的 SubTask 序列预览 */
  subtaskPreview: SubTaskPreview[];
}

export interface SubTaskPreview {
  step: number;
  action: string;
  target: string;
  /** 对应 SubTask 的 offset1 值 */
  offset1: number;
}

export interface ParseResult {
  success: boolean;
  task?: ParsedTask;
  error?: string;
  /** 多种可能的解析结果（模糊匹配时） */
  alternatives?: ParsedTask[];
}

// ========== 关键词词典 ==========

/** 取货关键词 */
const PICK_KEYWORDS = ["取货", "拿货", "取", "拿", "装货", "装载", "提货"];
/** 放货关键词 */
const DROP_KEYWORDS = ["放货", "卸货", "放", "卸", "放下", "送到", "送至", "搬到", "搬至", "运到", "运至"];
/** 搬运关键词（隐含取+放） */
const TRANSPORT_KEYWORDS = ["送", "搬", "运", "转运", "配送", "搬运", "运送", "转移"];
/** 充电关键词 */
const CHARGE_KEYWORDS = ["充电", "去充电", "回充", "补电"];
/** 控制关键词 */
const CONTROL_MAP: Record<string, "pause" | "resume" | "cancel" | "estop"> = {
  "暂停": "pause", "停一下": "pause", "等一下": "pause", "停": "pause",
  "继续": "resume", "恢复": "resume", "开始": "resume", "出发": "resume",
  "取消": "cancel", "取消任务": "cancel", "算了": "cancel", "不要了": "cancel",
  "急停": "estop", "紧急停止": "estop", "停车": "estop",
};

// ========== 点位别名映射 ==========
// 这个映射需要根据你们实际的地图点位来维护
// key: 自然语言中的名称, value: 系统中的点位ID

const POINT_ALIASES: Record<string, string> = {
  // Q点 = 取放货点（isPickOrDrop: true）
  "a货架": "Q001", "A货架": "Q001", "1号货架": "Q001",
  "b货架": "Q002", "B货架": "Q002", "2号货架": "Q002",
  "c货架": "Q003", "C货架": "Q003", "3号货架": "Q003",
  "a工位": "Q010", "A工位": "Q010", "1号工位": "Q010",
  "b工位": "Q011", "B工位": "Q011", "2号工位": "Q011",
  "c工位": "Q012", "C工位": "Q012", "3号工位": "Q012",
  // P点 = 路径点（isPickOrDrop: false），特殊功能点
  "充电站": "P100", "充电桩": "P100", "充电点": "P100",
  "待命点": "P200", "等待点": "P200", "待机点": "P200",
};

// ========== 语音识别谐音纠错 ==========

/** STT 常见谐音/错字修正表（key: 错误, value: 正确） */
const STT_FIXES: [RegExp, string][] = [
  // 数字谐音
  [/午/g, "五"], [/无/g, "五"], [/舞/g, "五"],
  [/刘/g, "六"], [/溜/g, "六"], [/留/g, "六"],
  [/期/g, "七"], [/漆/g, "七"], [/骑/g, "七"],
  [/吧/g, "八"], [/爸/g, "八"], [/把/g, "八"],
  [/就/g, "九"], [/酒/g, "九"],
  [/石/g, "十"], [/是/g, "十"],
  [/衣/g, "一"], [/已/g, "一"],
  [/耳/g, "二"], [/饿/g, "二"],
  [/山/g, "三"], [/伞/g, "三"],
  [/思/g, "四"], [/死/g, "四"],
  // 动词谐音
  [/道/g, "到"], [/倒/g, "到"],
  [/区/g, "去"], [/趣/g, "去"],
  [/拿/g, "那"],
  // 名词谐音
  [/掂/g, "点"], [/店/g, "点"], [/典/g, "点"],
  [/好/g, "号"], [/耗/g, "号"],
  [/或/g, "货"], [/活/g, "货"],
  [/冲/g, "充"], [/虫/g, "充"],
];

/** 对 STT 输出做谐音纠错，提高下游解析成功率 */
function sttFuzzyFix(text: string): string {
  let fixed = text;
  for (const [pattern, replacement] of STT_FIXES) {
    fixed = fixed.replace(pattern, replacement);
  }
  return fixed;
}

// ========== 核心解析逻辑 ==========

/**
 * 解析自然语言文本为结构化任务
 * 
 * 回内网后，这个函数的实现替换为：
 * ```
 * export async function parseText(text: string): Promise<ParseResult> {
 *   const resp = await fetch('http://内网大模型地址/api/parse', {
 *     method: 'POST',
 *     body: JSON.stringify({ text, schema: TASK_SCHEMA })
 *   });
 *   return resp.json();
 * }
 * ```
 */
export async function parseText(text: string): Promise<ParseResult> {
  const cleaned = sttFuzzyFix(text.trim());

  if (!cleaned) {
    return { success: false, error: "输入为空" };
  }

  // 1. 先检查是否是控制指令
  const controlResult = tryParseControl(cleaned);
  if (controlResult) return { success: true, task: controlResult };

  // 2. 检查是否是充电指令
  const chargeResult = tryParseCharge(cleaned);
  if (chargeResult) return { success: true, task: chargeResult };

  // 3. 检查是否是搬运指令（最复杂）
  const transportResult = tryParseTransport(cleaned);
  if (transportResult) return { success: true, task: transportResult };

  // 4. 检查是否是简单移动指令
  const moveResult = tryParseMove(cleaned);
  if (moveResult) return { success: true, task: moveResult };

  // 5. 都不匹配
  return {
    success: false,
    error: `无法理解指令："${cleaned}"。支持的格式：\n` +
      '• "把A货架的货送到B工位"\n' +
      '• "去1号点取货送到3号点"\n' +
      '• "去充电"\n' +
      '• "暂停" / "继续" / "取消"',
  };
}

// ========== 子解析器 ==========

function tryParseControl(text: string): ParsedTask | null {
  for (const [keyword, action] of Object.entries(CONTROL_MAP)) {
    if (text.includes(keyword)) {
      return {
        taskType: "control",
        controlAction: action,
        confidence: 0.95,
        rawText: text,
        description: `控制指令：${keyword}`,
        subtaskPreview: [],
      };
    }
  }
  return null;
}

function tryParseCharge(text: string): ParsedTask | null {
  if (CHARGE_KEYWORDS.some(k => text.includes(k))) {
    return {
      taskType: "charge",
      to: resolvePoint("充电站") || "P100",
      confidence: 0.9,
      rawText: text,
      description: "前往充电站充电",
      subtaskPreview: [
        { step: 1, action: "移动到充电站", target: "P100", offset1: 1 },
        { step: 2, action: "对接充电", target: "P100", offset1: 4 },
      ],
    };
  }
  return null;
}

function tryParseTransport(text: string): ParsedTask | null {
  // 模式1: "把A的货送到B" / "将A的货搬到B" / "把A的货放到B"
  const pattern1 = /[把将从](.+?)(?:的货|的东西|的物料|的物品|货物)?(?:送|搬|运|转|配送|放)(?:到|至|去)(.+)/;
  const m1 = text.match(pattern1);
  if (m1) {
    return buildTransportTask(m1[1].trim(), m1[2].trim(), text);
  }

  // 模式2: "去A取货送到B" / "到A拿货搬到B"
  const pattern2 = /(?:去|到|前往)(.+?)(?:取货|拿货|取|拿|装货).*?(?:送|搬|运|放)(?:到|至|去)(.+)/;
  const m2 = text.match(pattern2);
  if (m2) {
    return buildTransportTask(m2[1].trim(), m2[2].trim(), text);
  }

  // 模式3: "从A搬到B" / "从A运到B"
  const pattern3 = /从(.+?)(?:搬|运|送|转)(?:到|至|去)(.+)/;
  const m3 = text.match(pattern3);
  if (m3) {
    return buildTransportTask(m3[1].trim(), m3[2].trim(), text);
  }

  // 模式4: "A到B" 简写（如 "1号点到3号点"）
  const pattern4 = /(.+?)(?:到|至|→|->)(.+)/;
  const m4 = text.match(pattern4);
  if (m4 && TRANSPORT_KEYWORDS.some(k => text.includes(k))) {
    return buildTransportTask(m4[1].trim(), m4[2].trim(), text);
  }

  // 模式5: "去A把货放到B" / "去A把货送到B"
  const pattern5 = /(?:去|到|前往)(.+?)(?:把货|将货|把东西|货)(?:放|送|搬|运)(?:到|至|去)(.+)/;
  const m5 = text.match(pattern5);
  if (m5) {
    return buildTransportTask(m5[1].trim(), m5[2].trim(), text);
  }

  return null;
}

function tryParseMove(text: string): ParsedTask | null {
  // "去X点" / "前往X" / "移动到X"
  const pattern = /(?:去|前往|移动到|到)(.+?)(?:点|$)/;
  const m = text.match(pattern);
  if (m) {
    const target = m[1].trim();
    const pointId = resolvePoint(target);
    if (pointId) {
      return {
        taskType: "move",
        to: pointId,
        confidence: 0.75,
        rawText: text,
        description: `移动到 ${target}（${pointId}）`,
        subtaskPreview: [
          { step: 1, action: `移动到${target}`, target: pointId, offset1: 1 },
        ],
      };
    }
  }
  return null;
}

// ========== 辅助函数 ==========

function buildTransportTask(fromRaw: string, toRaw: string, rawText: string): ParsedTask | null {
  // 搬运任务的起点和终点都是取放货点（Q点）
  const fromId = resolvePoint(fromRaw, "pickdrop");
  const toId = resolvePoint(toRaw, "pickdrop");

  if (!fromId && !toId) return null;

  const fromLabel = fromId || `未识别(${fromRaw})`;
  const toLabel = toId || `未识别(${toRaw})`;
  const confidence = (fromId ? 0.45 : 0) + (toId ? 0.45 : 0);

  return {
    taskType: "transport",
    from: fromId || undefined,
    to: toId || undefined,
    needPick: true,
    needDrop: true,
    confidence,
    rawText,
    description: `从 ${fromRaw}（${fromLabel}）取货 → 送到 ${toRaw}（${toLabel}）`,
    subtaskPreview: [
      { step: 1, action: `移动到${fromRaw}`, target: fromLabel, offset1: 1 },
      { step: 2, action: "取货", target: fromLabel, offset1: 9 },
      { step: 3, action: `移动到${toRaw}`, target: toLabel, offset1: 1 },
      { step: 4, action: "放货", target: toLabel, offset1: 3 },
    ],
  };
}

// 中文数字转阿拉伯数字
const CN_NUM: Record<string, number> = {
  "零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
  "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10,
};

function chineseToNumber(cn: string): number | null {
  // 单字："六" → 6
  if (cn.length === 1 && CN_NUM[cn] !== undefined) return CN_NUM[cn];
  // "十二" → 12, "二十" → 20, "二十三" → 23
  if (cn.includes("十")) {
    const parts = cn.split("十");
    const tens = parts[0] ? (CN_NUM[parts[0]] ?? 1) : 1;
    const ones = parts[1] ? (CN_NUM[parts[1]] ?? 0) : 0;
    return tens * 10 + ones;
  }
  return null;
}

/**
 * 解析点位名称为系统点位ID
 *
 * 点位体系：
 *   Q + 数字 = 取放货点（isPickOrDrop: true），如 Q001, Q006
 *   P + 数字 = 路径点（isPickOrDrop: false），如 P001, P100
 *
 * role 参数决定默认前缀：
 *   "pickdrop" → 默认生成 Q 点（搬运任务的起点/终点）
 *   "path"     → 默认生成 P 点（移动任务、充电等路径点）
 */
function resolvePoint(name: string, role: "pickdrop" | "path" = "path"): string | null {
  // 1. 先查别名映射（别名已包含正确的 Q/P 前缀）
  if (POINT_ALIASES[name]) return POINT_ALIASES[name];

  const prefix = role === "pickdrop" ? "Q" : "P";

  // 2. 匹配阿拉伯数字模式："5号" → Q005/P005
  const numMatch = name.match(/(\d+)(?:号)?(?:点|号)?/);
  if (numMatch) {
    const num = parseInt(numMatch[1], 10);
    return `${prefix}${num.toString().padStart(3, "0")}`;
  }

  // 3. 匹配中文数字模式："六号点" → Q006/P006
  const cnMatch = name.match(/([零一二两三四五六七八九十]+)(?:号)?(?:点|号)?/);
  if (cnMatch) {
    const num = chineseToNumber(cnMatch[1]);
    if (num !== null) return `${prefix}${num.toString().padStart(3, "0")}`;
  }

  // 4. 如果已经是 Q/Pxxx 格式，直接返回
  if (/^[QP]\d{3,}$/i.test(name)) return name.toUpperCase();

  return null;
}

// ========== 用于配置热更新的接口 ==========

/**
 * 动态更新点位别名（可以从数据库加载）
 */
export function updatePointAliases(aliases: Record<string, string>): void {
  Object.assign(POINT_ALIASES, aliases);
}

/**
 * 获取当前点位别名表（供前端展示）
 */
export function getPointAliases(): Record<string, string> {
  return { ...POINT_ALIASES };
}
