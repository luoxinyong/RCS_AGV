import { pg } from "../services/postgres";
import type { User } from "../types/user";

/** 查询用户基础信息 */
export async function getUserInfo(username: string, password: string) {
  const result = await pg.query(
    `SELECT * FROM users WHERE employee_no = $1 AND password = $2;`,
    [username, password]
  );
  return result.rows as User[];
}
