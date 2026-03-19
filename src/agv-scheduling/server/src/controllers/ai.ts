/**
 * AI 交互控制器
 */
import { Request, Response } from "express";
import { parseText, getPointAliases, updatePointAliases } from "../services/ai-parser";
import STTService from "../services/stt";

/**
 * POST /api/v1/ai/parse
 * 解析自然语言文本为结构化任务
 */
export const aiParse = async (req: Request, res: Response): Promise<void> => {
  try {
    const { text } = req.body;

    if (!text || typeof text !== "string") {
      res.status(400).json({ success: false, error: "缺少 text 字段" });
      return;
    }

    const result = await parseText(text);
    res.json(result);
  } catch (error: any) {
    console.error("[AI Parse Error]", error);
    res.status(500).json({ success: false, error: "解析服务异常" });
  }
};

/**
 * POST /api/v1/ai/confirm
 * 确认执行解析后的任务（调用现有的任务下发接口）
 */
export const aiConfirm = async (req: Request, res: Response): Promise<void> => {
  try {
    const { task } = req.body;

    if (!task) {
      res.status(400).json({ success: false, error: "缺少 task 字段" });
      return;
    }

    // TODO: 回内网后对接真实的任务下发
    console.log("[AI Confirm] 任务确认下发:", JSON.stringify(task, null, 2));

    res.json({
      success: true,
      message: "任务已确认下发（本地模拟）",
      taskId: `T${Date.now()}`,
      task,
    });
  } catch (error: any) {
    console.error("[AI Confirm Error]", error);
    res.status(500).json({ success: false, error: "任务下发异常" });
  }
};

/**
 * GET /api/v1/ai/aliases
 * 获取点位别名映射表
 */
export const getAliases = async (_req: Request, res: Response): Promise<void> => {
  res.json({ success: true, aliases: getPointAliases() });
};

/**
 * PUT /api/v1/ai/aliases
 * 更新点位别名映射表
 */
export const putAliases = async (req: Request, res: Response): Promise<void> => {
  const { aliases } = req.body;
  if (!aliases || typeof aliases !== "object") {
    res.status(400).json({ success: false, error: "缺少 aliases 字段" });
    return;
  }
  updatePointAliases(aliases);
  res.json({ success: true, message: "别名已更新" });
};

// ========== 语音相关接口 ==========

/**
 * POST /api/v1/ai/voice
 * 上传音频文件 → STT 转写 → 文本解析
 *
 * 接入方式：
 *   curl -X POST http://localhost:3001/api/v1/ai/voice \
 *     -F "audio=@recording.wav" \
 *     -F "format=wav"
 */
export const aiVoice = async (req: Request, res: Response): Promise<void> => {
  try {
    const file = req.file;
    if (!file) {
      res.status(400).json({ success: false, error: "缺少音频文件（字段名: audio）" });
      return;
    }

    const format = (req.body.format as string) || "wav";
    const language = (req.body.language as string) || "zh";

    // 1. STT 转写
    const stt = STTService.getInstance();
    const sttResult = await stt.transcribe(file.buffer, { format, language });

    if (!sttResult.success || !sttResult.text) {
      res.json({
        success: false,
        error: sttResult.error || "语音识别失败",
        stt: { backend: sttResult.backend, duration: sttResult.duration },
      });
      return;
    }

    // 2. 文本解析
    const parseResult = await parseText(sttResult.text);

    res.json({
      ...parseResult,
      stt: {
        text: sttResult.text,
        confidence: sttResult.confidence,
        backend: sttResult.backend,
        duration: sttResult.duration,
      },
    });
  } catch (error: any) {
    console.error("[AI Voice Error]", error);
    res.status(500).json({ success: false, error: "语音处理异常" });
  }
};

/**
 * POST /api/v1/ai/voice/text
 * 接收外部 STT 工具已转写好的文本 → 文本解析
 *
 * 适用场景：
 *   - VOSK/Whisper.cpp 独立运行，转写完成后把文本推给这个接口
 *   - 前端使用浏览器 Web Speech API 转写后发送
 *   - ROS 节点中的语音识别模块输出文本
 *
 * 请求体：{ "text": "把A货架的货送到B工位", "source": "vosk" }
 */
export const aiVoiceText = async (req: Request, res: Response): Promise<void> => {
  try {
    const { text, source } = req.body;

    if (!text || typeof text !== "string") {
      res.status(400).json({ success: false, error: "缺少 text 字段" });
      return;
    }

    const parseResult = await parseText(text.trim());

    res.json({
      ...parseResult,
      stt: { text: text.trim(), source: source || "external", backend: "passthrough" },
    });
  } catch (error: any) {
    console.error("[AI VoiceText Error]", error);
    res.status(500).json({ success: false, error: "解析服务异常" });
  }
};

/**
 * GET /api/v1/ai/voice/status
 * 查询 STT 后端状态
 */
export const aiVoiceStatus = async (_req: Request, res: Response): Promise<void> => {
  const stt = STTService.getInstance();
  const info = stt.getBackendInfo();
  const healthy = await stt.healthCheck();

  res.json({
    success: true,
    backend: info.name,
    healthy,
  });
};
