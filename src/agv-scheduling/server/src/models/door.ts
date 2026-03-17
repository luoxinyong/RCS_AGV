import { pg } from "../services/postgres";
import type { Door } from "../types/door";

export async function queryDoors(): Promise<Door[]> {
  const result = await pg.query("SELECT * FROM doors");
  return result.rows;
}
