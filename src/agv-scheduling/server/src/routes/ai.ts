/**
 * AI 交互路由
 */
import { Router } from "express";
import multer from "multer";
import {
  aiParse,
  aiConfirm,
  getAliases,
  putAliases,
  aiVoice,
  aiVoiceText,
  aiVoiceStatus,
} from "../controllers/ai";

const router = Router();

// multer 配置：音频文件存内存（不落盘），限制 10MB
const upload = multer({
  storage: multer.memoryStorage(),
  limits: { fileSize: 10 * 1024 * 1024 },
});

// ===== 文本解析 =====

/** 解析自然语言文本 */
router.post("/parse", aiParse);

/** 确认执行任务 */
router.post("/confirm", aiConfirm);

// ===== 点位别名 =====

/** 获取/更新点位别名 */
router.get("/aliases", getAliases);
router.put("/aliases", putAliases);

// ===== 语音接口 =====

/** 上传音频 → STT 转写 → 文本解析（一站式） */
router.post("/voice", upload.single("audio"), aiVoice);

/** 接收已转写的文本 → 文本解析（外部 STT 工具推送用） */
router.post("/voice/text", aiVoiceText);

/** 查询 STT 后端状态 */
router.get("/voice/status", aiVoiceStatus);

export default router;
