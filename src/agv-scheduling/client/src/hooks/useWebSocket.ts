import { useEffect, useRef, useState, useCallback } from "react";

interface WebSocketHandlers {
  onOpen?: () => void;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  onMessage?: (data: any) => void;
  onClose?: () => void;
  onError?: (error: Event) => void;
}

interface WebSocketOptions {
  reconnectEnabled: boolean;
  reconnectInterval: number;
  reconnectAttempts: number;
  manualConnect: boolean; // 是否手动触发连接
}

const useWebSocket = (
  url: string,
  handlers?: WebSocketHandlers,
  options: WebSocketOptions = {
    reconnectEnabled: true,
    reconnectInterval: 3000,
    reconnectAttempts: Infinity,
    manualConnect: false, // 是否手动触发连接
  }
) => {
  const [isReady, setIsReady] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectCountRef = useRef(0);
  const reconnectTimerRef = useRef<NodeJS.Timeout | null>(null);
  const optionsRef = useRef(options);
  const handlersRef = useRef(handlers);

  // 更新 handlers 引用
  useEffect(() => {
    handlersRef.current = handlers;
  }, [handlers]);

  // 更新 options 引用
  useEffect(() => {
    optionsRef.current = options;
  }, [options]);

  // 关闭连接
  const closeConnection = useCallback(() => {
    if (wsRef.current) {
      wsRef.current.onclose = null; // 防止触发 onclose 事件
      wsRef.current.close();
      wsRef.current = null;
    }
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }
    setIsReady(false);
    reconnectCountRef.current = 0;
  }, []);

  // 建立连接
  const connect = useCallback(() => {
    // 如果已有连接且状态正常，直接返回
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      setIsReady(true);
      return;
    }

    closeConnection(); // 先关闭现有连接

    wsRef.current = new WebSocket(url);

    wsRef.current.onopen = () => {
      setIsReady(true);
      reconnectCountRef.current = 0;
      handlersRef.current?.onOpen?.();
    };

    wsRef.current.onmessage = (event) => {
      let parsedData;
      try {
        parsedData = JSON.parse(event.data);
      } catch {
        parsedData = event.data;
      }
      handlersRef.current?.onMessage?.(parsedData);
    };

    wsRef.current.onclose = () => {
      setIsReady(false);
      handlersRef.current?.onClose?.();

      // 自动重连逻辑
      if (
        optionsRef.current.reconnectEnabled &&
        reconnectCountRef.current < optionsRef.current.reconnectAttempts
      ) {
        reconnectTimerRef.current = setTimeout(() => {
          reconnectCountRef.current++;
          connect();
        }, optionsRef.current.reconnectInterval);
      }
    };

    wsRef.current.onerror = (error) => {
      handlersRef.current?.onError?.(error);
    };
  }, [url, closeConnection]);

  // 发送消息
  const sendMessage = useCallback((message: unknown) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const messageToSend =
        typeof message === "string" ? message : JSON.stringify(message);
      wsRef.current.send(messageToSend);
    } else {
      console.warn("WebSocket is not connected. Message not sent:", message);
    }
  }, []);

  // 初始化
  useEffect(() => {
    if (optionsRef.current.manualConnect === false) {
      connect();
    }
    return () => {
      closeConnection();
    };
  }, [connect, closeConnection]);

  return {
    isReady,
    connect,
    close: closeConnection,
    sendMessage,
  };
};

export default useWebSocket;
