import { Request, Response, NextFunction } from "express";
import { insertLog } from "../models/logger";

export function requestLogger() {
  return async (req: Request, res: Response, next: NextFunction) => {
    const requestAt = new Date();
    const { method, originalUrl: path, query, body } = req;

    // 拦截 res.json 方法
    const originalJson = res.json.bind(res);
    let responseData: any = null;
    res.json = (data: any) => {
      responseData = data;
      return originalJson(data);
    };

    // 响应完成时记录日志
    res.on("finish", () => {
      try {
        insertLog({
          method,
          path,
          status: res.statusCode,
          duration: new Date().getTime() - requestAt.getTime(),
          auth: (req as any).auth, // 通过JWT中间件设置的用户信息
          query,
          body,
          responseData,
        });
      } catch (err) {
        console.error("[API Log Write Failed]", err);
      }
    });

    next();
  };
}
