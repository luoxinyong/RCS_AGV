import { pg } from "../services/postgres";
import type { TaskRecord } from "../types/task";
import type { Pagination } from "../types/pagination";

export async function queryTaskLogs(args: {
  page: number;
  pageSize: number;
  typeFilters: string;
  sorterField?: string;
  ascend: boolean;
}): Promise<Pagination<TaskRecord>> {
  const { page, pageSize, typeFilters, sorterField, ascend } = args;

  const filters =
    typeFilters.length > 0
      ? typeFilters
          .split(",")
          .map((e) => `'${e}'`)
          .join(",")
      : null;
  const result = await pg.query(
    `SELECT a.*, b."name" as agv_name, COUNT(*) OVER() as total_count 
     FROM tasks a LEFT JOIN devices b on a.agv_id = b.id ${
       filters ? `WHERE a.type in (${filters})` : ""
     } ${
      sorterField ? `ORDER BY ${sorterField} ${ascend ? "ASC" : "DESC"}` : ""
    }
     LIMIT $1 OFFSET $2`,
    [pageSize, (page - 1) * pageSize]
  );

  // 提取数据
  const data = result.rows;
  return {
    data: data.map((e) => {
      const { total_count, ...rest } = e; // 移除total_count字段
      rest.body = rest.body ? JSON.stringify(rest.body) : "";
      return rest;
    }),
    total: data.length > 0 ? parseInt(data[0].total_count) : 0,
    page,
    pageSize,
  };
}

export async function insertTask(args: {
  agv_id: string;
  type: string;
  duration: number;
  code: string | null;
  body: any;
}) {
  const { agv_id, type, duration, code, body } = args;
  const result = await pg.query(
    "INSERT INTO tasks (agv_id, type, duration, code, body) VALUES ($1, $2, $3, $4, $5)",
    [agv_id, type, duration, code, body]
  );
  return result;
}
