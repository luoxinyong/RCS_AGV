import { Request, Response, NextFunction } from "express";
import { ZodSchema } from "zod";

export function validate(
  schema: ZodSchema<any>,
  type: "body" | "query" | "params" = "body"
) {
  return (req: Request, res: Response, next: NextFunction) => {
    const result = schema.safeParse(req[type]);
    if (result.success) {
      // 校验通过，继续执行
      next();
    } else {
      // 校验失败，返回错误信息
      res.json({
        status: 500,
        statusText: result.error.errors.map((e) => e.message).join(", "),
      });
    }
  };
}
