import { pg } from "../services/postgres";

/** 插入接口请求日志 */
export async function insertLog(log: {
  method: string; // 请求方法
  path: string; // 请求路径
  status: number; // 响应状态
  duration: number; // 耗时
  auth: any; // 用户信息
  query: any; // 请求参数
  body: any; // 请求体
  responseData: any; // 响应数据
}) {
  const { method, path, status, duration, auth, query, body, responseData } =
    log;
  const result = await pg.query(
    "INSERT INTO logs (method, path, status, duration, auth, query, body, response) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)",
    [method, path, status, duration, auth, query, body, responseData]
  );
  return result;
}
