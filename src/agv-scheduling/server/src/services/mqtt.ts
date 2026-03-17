import mqtt, {
  MqttClient as MQTTClient,
  IClientOptions,
  IClientPublishOptions,
  IClientSubscribeOptions,
} from "mqtt";

export default class MqttClient {
  private static instance: MqttClient | null = null;
  private client?: MQTTClient;
  private connected: boolean = false;
  private subscriptions: Map<
    string,
    {
      options: IClientSubscribeOptions;
      callback?: (topic: string, message: Buffer) => void;
    }
  > = new Map();

  // 获取单例
  public static getInstance(): MqttClient {
    if (!MqttClient.instance) {
      MqttClient.instance = new MqttClient();
    }
    return MqttClient.instance;
  }

  // 私有构造，外部通过 getInstance 获取单例
  private constructor() {}

  /** 配置并建立与 broker 的连接，通常只需要调用一次 */
  public configure(args: {
    brokerUrl: string;
    options?: IClientOptions;
    onConnected?: () => void;
  }) {
    const {
      brokerUrl,
      options = { reconnectPeriod: 5000 },
      onConnected,
    } = args;

    if (!brokerUrl) {
      throw new Error("brokerUrl is required to configure MQTT client.");
    }

    // 清理旧连接
    if (this.connected) {
      this.disconnect();
    }

    // 建立新连接
    this.client = mqtt.connect(brokerUrl, options);

    this.client.on("connect", () => {
      this.connected = true;
      console.log("✅ MQTT connected:", brokerUrl);
      this.resubscribeAll(); // 重连后重新订阅之前的主题
      onConnected?.();
    });

    this.client.on("close", () => {
      this.connected = false;
    });

    this.client.on("offline", () => {
      this.connected = false;
    });
  }

  // 订阅主题
  subscribe(args: {
    topic: string;
    options?: IClientSubscribeOptions;
    callback?: (topic: string, message: Buffer) => void;
  }) {
    if (!this.client) return;

    const { topic, options = { qos: 0 }, callback } = args;
    this.client.subscribe(topic, options, (err) => {
      if (err) {
        console.error("❌ MQTT subscription error:", err);
      } else {
        console.log(`📌 Subscribed to topic: ${topic}`);
        this.subscriptions.set(topic, { options, callback });
      }
    });

    if (callback) {
      this.client.on("message", callback);
    }
  }

  // 取消订阅
  unsubscribe(topic: string) {
    this.client?.unsubscribe(topic, (err) => {
      if (err) {
        console.error("❌ MQTT unsubscribe error:", err);
      } else {
        console.log(`🚫 Unsubscribed from topic: ${topic}`);
        this.subscriptions.delete(topic);
      }
    });
  }

  // 发布消息
  publish(args: {
    topic: string;
    options?: IClientPublishOptions;
    message: string | Buffer;
  }) {
    const { topic, message, options = { qos: 0, retain: false } } = args;
    this.client?.publish(topic, message, options, (err) => {
      if (err) {
        console.error("❌ MQTT publish error:", err);
      } else {
        console.log(`📤 Published message to ${topic}`);
      }
    });
  }

  // 主动断开
  disconnect() {
    try {
      this.client?.end(true);
    } catch (e) {}
    this.connected = false;
  }

  // 手动触发重连
  reconnect() {
    try {
      this.client?.reconnect();
    } catch (e) {}
  }

  // 获取状态
  isConnected(): boolean {
    return this.connected;
  }

  // 重连后恢复订阅
  private resubscribeAll() {
    if (this.subscriptions.size > 0) {
      for (const [topic, { options, callback }] of this.subscriptions) {
        this.subscribe({ topic, options, callback });
      }
    }
  }
}
