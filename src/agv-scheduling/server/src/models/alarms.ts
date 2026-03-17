import { pg } from "../services/postgres";
import type { AlarmRecord } from "../types/alarms";
import type { Pagination } from "../types/pagination";

export async function queryAlarms(args: {
  page: number;
  pageSize: number;
  sorterField?: string;
  ascend: boolean;
}): Promise<Pagination<AlarmRecord>> {
  const { page, pageSize, sorterField, ascend } = args;

  const result = await pg.query(
    `SELECT a.*, b."name" as agv_name, COUNT(*) OVER() as total_count 
     FROM alarms a LEFT JOIN devices b on a.agv_id = b.id ${
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

export async function insertAlarm(args: {
  agv_id: string;
  tr_check_state: number;
  tr_check_error: number;
  err1: number;
  err2: number;
  err3: number;
  err4: number;
  err5: number;
  warnings: number;
  code: string;
  body: any;
}) {
  const result = await pg.query(
    `INSERT INTO alarms (agv_id, tr_check_state, tr_check_error, err1, err2, err3, err4, err5, warnings, code, body) 
     VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11)`,
    [
      args.agv_id,
      args.tr_check_state,
      args.tr_check_error,
      args.err1,
      args.err2,
      args.err3,
      args.err4,
      args.err5,
      args.warnings,
      args.code,
      args.body,
    ]
  );
  return result;
}
