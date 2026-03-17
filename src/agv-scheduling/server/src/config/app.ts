import dotenv from "dotenv";

// 加载环境变量
dotenv.config();

const APP_CONFIG = {
  PORT: process.env.PORT || 3001,
  API_VERSION: "v1",
  NODE_ENV: process.env.NODE_ENV || "development",
  JWT_SECRET: process.env.JWT_SECRET || "",

  MQTT_HOST: process.env.MQTT_HOST!,
  MQTT_PORT: process.env.MQTT_PORT!,

  PG_HOST: process.env.PG_HOST!,
  PG_PORT: process.env.PG_PORT!,
  PG_USER: process.env.PG_USER!,
  PG_PASSWORD: process.env.PG_PASSWORD!,
  PG_DATABASE: process.env.PG_DATABASE!,
};

export default APP_CONFIG;
