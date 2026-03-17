import { Request, Response, NextFunction } from "express";

export function errorHandler(
  err: any,
  req: Request,
  res: Response,
  next: NextFunction
) {
  res.json({
    status: err.status || 500,
    statusText: err.message || "服务器内部错误",
  });
}
