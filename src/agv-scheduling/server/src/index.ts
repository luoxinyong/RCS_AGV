import cors from "cors";
import http from "http";
import express from "express";
import { expressjwt } from "express-jwt";
import APP_CONFIG from "./config/app";
import userRoutes from "./routes/user";
import deviceRoutes from "./routes/device";
import alarmRoutes from "./routes/alarms";
import taskRoutes from "./routes/tasks";
import doorRoutes from "./routes/door";
import mapRoutes from "./routes/map";
import { initPostgres } from "./services/postgres";
import { errorHandler } from "./middlewares/error";
import { requestLogger } from "./middlewares/logger";
import { startScheduling } from "./services/scheduler";

// Express应用设置
const app = express();
const server = http.createServer(app);

// 基础中间件
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));
app.use(requestLogger());

// token校验
app.use(
  expressjwt({ secret: APP_CONFIG.JWT_SECRET, algorithms: ["HS256"] }).unless({
    path: [
      { url: `/api/${APP_CONFIG.API_VERSION}/user/login`, method: "POST" },
    ],
  })
);

// API路由 (添加版本控制)
app.use(`/api/${APP_CONFIG.API_VERSION}/user`, userRoutes);
app.use(`/api/${APP_CONFIG.API_VERSION}/device`, deviceRoutes);
app.use(`/api/${APP_CONFIG.API_VERSION}/task`, taskRoutes);
app.use(`/api/${APP_CONFIG.API_VERSION}/alarm`, alarmRoutes);
app.use(`/api/${APP_CONFIG.API_VERSION}/map`, mapRoutes);
app.use(`/api/${APP_CONFIG.API_VERSION}/doors`, doorRoutes);

// 错误处理中间件
app.use(errorHandler);

// 退出处理
const shutdown = () => {
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(1), 15000);
};
process.on("SIGTERM", shutdown);
process.on("SIGINT", shutdown);

// 启动服务器
(async function () {
  try {
    // 初始化数据库
    await initPostgres();

    // 开启调度
    await startScheduling(server);

    server.listen(APP_CONFIG.PORT, () => {
      console.log(
        `✅ Server is running in "${APP_CONFIG.NODE_ENV}" mode at http://localhost:${APP_CONFIG.PORT}`
      );
      console.log(`📚 API Version: ${APP_CONFIG.API_VERSION}`);
    });
  } catch (error) {
    console.error("❌ 服务器启动失败:", error);
    process.exit(1);
  }
})();

export default app;
