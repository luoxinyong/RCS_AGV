import { Server } from "http";
import WebSocket, { WebSocketServer, RawData } from "ws";

export default class WebSocketClient {
  private static instance: WebSocketClient | null = null;
  private wsClients = new Set<WebSocket>();
  private wss?: WebSocketServer;

  // 私有构造，外部通过 getInstance 获取单例
  private constructor() {}

  public static getInstance(): WebSocketClient {
    if (!WebSocketClient.instance) {
      WebSocketClient.instance = new WebSocketClient();
    }
    return WebSocketClient.instance;
  }

  /**
   * 初始化 WebSocket 服务，通常只需要调用一次
   * 调用示例：
   *   WebSocketClient.getInstance().initialize(server, (ws, data) => { ... });
   */
  public initialize(
    server: Server,
    options: { onMessage: (ws: WebSocket, data: RawData) => void }
  ) {
    this.stop(); // 清理旧服务
    this.wss = new WebSocketServer({ server });
    this.wss.on("connection", (ws) => {
      this.wsClients.add(ws);

      // 接收消息事件
      ws.on("message", (data) => options.onMessage(ws, data));

      // 关闭连接事件
      ws.on("close", () => this.wsClients.delete(ws));
    });
  }

  /** 广播消息给所有连接的客户端 */
  public broadcastData(args: { type: string; data: any }) {
    for (const client of this.wsClients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify(args));
      }
    }
  }

  /** 发送消息给指定的客户端 */
  public sendData(ws: WebSocket, args: { type: string; data: any }) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(args));
    }
  }

  /** 停止服务并清理 */
  public stop() {
    try {
      for (const ws of this.wsClients) {
        ws.terminate();
      }
      this.wsClients.clear();
      if (this.wss) {
        this.wss.close();
      }
    } catch (e) {}
  }
}
