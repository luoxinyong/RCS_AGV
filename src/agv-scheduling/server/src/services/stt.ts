/**
 * 语音转文本（STT）服务接口
 *
 * 可插拔设计：当前支持三种后端，通过环境变量 STT_BACKEND 切换：
 *   - "vosk"        → 调用本地 VOSK HTTP 服务（默认 localhost:2700）
 *   - "whisper"     → 调用本地 Whisper.cpp HTTP 服务（默认 localhost:8080）
 *   - "passthrough" → 不做语音识别，直接返回前端已转写的文本（默认值）
 *
 * 使用方式：
 *   1. 前端录音 → POST /api/v1/ai/voice（音频文件）→ STT 转写 → parseText → 返回结果
 *   2. 外部 STT 工具直接推送文本 → POST /api/v1/ai/voice/text → parseText → 返回结果
 *   3. 前端用浏览器 Web Speech API 转写 → 和方式2相同
 *
 * VOSK 服务启动示例：
 *   docker run -d -p 2700:2700 alphacep/kaldi-zh:latest
 *   或：vosk-server --model-path /path/to/model --port 2700
 *
 * Whisper.cpp 服务启动示例：
 *   ./server -m models/ggml-base.bin --host 0.0.0.0 --port 8080
 */

import APP_CONFIG from "../config/app";

// ========== 类型定义 ==========

export interface STTResult {
  /** 是否成功 */
  success: boolean;
  /** 转写后的文本 */
  text?: string;
  /** 置信度 0~1 */
  confidence?: number;
  /** 错误信息 */
  error?: string;
  /** STT 后端标识 */
  backend: string;
  /** 转写耗时（毫秒） */
  duration?: number;
}

export interface STTBackend {
  /** 后端名称 */
  name: string;
  /** 将音频 Buffer 转写为文本 */
  transcribe(audio: Buffer, options?: STTOptions): Promise<STTResult>;
  /** 检查后端是否可用 */
  healthCheck(): Promise<boolean>;
}

export interface STTOptions {
  /** 音频格式：wav, pcm, webm, ogg（默认 wav） */
  format?: string;
  /** 采样率（默认 16000） */
  sampleRate?: number;
  /** 语言（默认 zh） */
  language?: string;
}

// ========== VOSK 后端 ==========

class VoskBackend implements STTBackend {
  name = "vosk";
  private url: string;

  constructor() {
    this.url = APP_CONFIG.STT_VOSK_URL;
  }

  async transcribe(audio: Buffer, options?: STTOptions): Promise<STTResult> {
    const start = Date.now();
    try {
      const res = await fetch(this.url, {
        method: "POST",
        headers: {
          "Content-Type": `audio/${options?.format || "wav"}`,
        },
        body: audio,
      });

      if (!res.ok) {
        return { success: false, error: `VOSK 返回 ${res.status}`, backend: this.name };
      }

      const data = await res.json();
      // VOSK HTTP 服务返回格式：{ text: "识别结果" }
      const text = (data.text || "").trim();
      if (!text) {
        return { success: false, error: "VOSK 未识别到语音", backend: this.name };
      }

      return {
        success: true,
        text,
        confidence: data.confidence ?? 0.8,
        backend: this.name,
        duration: Date.now() - start,
      };
    } catch (err: any) {
      return {
        success: false,
        error: `VOSK 连接失败: ${err.message}`,
        backend: this.name,
      };
    }
  }

  async healthCheck(): Promise<boolean> {
    try {
      const res = await fetch(this.url.replace(/\/\s*$/, ""), { method: "GET" });
      return res.ok;
    } catch {
      return false;
    }
  }
}

// ========== Whisper.cpp 后端 ==========

class WhisperBackend implements STTBackend {
  name = "whisper";
  private url: string;

  constructor() {
    this.url = APP_CONFIG.STT_WHISPER_URL;
  }

  async transcribe(audio: Buffer, options?: STTOptions): Promise<STTResult> {
    const start = Date.now();
    try {
      // Whisper.cpp server 的 /inference 接口接收 multipart/form-data
      const formData = new FormData();
      const blob = new Blob([audio], { type: `audio/${options?.format || "wav"}` });
      formData.append("file", blob, `audio.${options?.format || "wav"}`);
      formData.append("language", options?.language || "zh");
      formData.append("response_format", "json");

      const res = await fetch(`${this.url}/inference`, {
        method: "POST",
        body: formData,
      });

      if (!res.ok) {
        return { success: false, error: `Whisper 返回 ${res.status}`, backend: this.name };
      }

      const data = await res.json();
      // Whisper.cpp server 返回格式：{ text: "识别结果" }
      const text = (data.text || "").trim();
      if (!text) {
        return { success: false, error: "Whisper 未识别到语音", backend: this.name };
      }

      return {
        success: true,
        text,
        confidence: 0.85,
        backend: this.name,
        duration: Date.now() - start,
      };
    } catch (err: any) {
      return {
        success: false,
        error: `Whisper 连接失败: ${err.message}`,
        backend: this.name,
      };
    }
  }

  async healthCheck(): Promise<boolean> {
    try {
      const res = await fetch(this.url, { method: "GET" });
      return res.ok;
    } catch {
      return false;
    }
  }
}

// ========== Passthrough 后端（不做 STT，直接传递文本） ==========

class PassthroughBackend implements STTBackend {
  name = "passthrough";

  async transcribe(_audio: Buffer): Promise<STTResult> {
    return {
      success: false,
      error: "Passthrough 模式不支持音频转写，请直接发送文本",
      backend: this.name,
    };
  }

  async healthCheck(): Promise<boolean> {
    return true; // 始终可用
  }
}

// ========== STT 服务（单例） ==========

class STTService {
  private static instance: STTService | null = null;
  private backend: STTBackend;

  public static getInstance(): STTService {
    if (!STTService.instance) {
      STTService.instance = new STTService();
    }
    return STTService.instance;
  }

  private constructor() {
    const backendType = APP_CONFIG.STT_BACKEND;
    switch (backendType) {
      case "vosk":
        this.backend = new VoskBackend();
        console.log(`🎤 STT 后端: VOSK (${APP_CONFIG.STT_VOSK_URL})`);
        break;
      case "whisper":
        this.backend = new WhisperBackend();
        console.log(`🎤 STT 后端: Whisper.cpp (${APP_CONFIG.STT_WHISPER_URL})`);
        break;
      default:
        this.backend = new PassthroughBackend();
        console.log("🎤 STT 后端: Passthrough（需前端或外部工具转写文本）");
    }
  }

  /** 转写音频为文本 */
  async transcribe(audio: Buffer, options?: STTOptions): Promise<STTResult> {
    return this.backend.transcribe(audio, options);
  }

  /** 获取当前后端信息 */
  getBackendInfo(): { name: string } {
    return { name: this.backend.name };
  }

  /** 检查 STT 后端是否可用 */
  async healthCheck(): Promise<boolean> {
    return this.backend.healthCheck();
  }

  /** 运行时切换后端（热切换，供调试用） */
  switchBackend(type: "vosk" | "whisper" | "passthrough"): void {
    switch (type) {
      case "vosk":
        this.backend = new VoskBackend();
        break;
      case "whisper":
        this.backend = new WhisperBackend();
        break;
      default:
        this.backend = new PassthroughBackend();
    }
    console.log(`🎤 STT 后端已切换为: ${type}`);
  }
}

export default STTService;
