import { pg } from "../services/postgres";

export async function queryMaps(): Promise<any> {
  const bdResult = await pg.query("SELECT * FROM buildings");
  const flResult = await pg.query("SELECT * FROM floors");
  // 将楼层添加到楼栋floors属性
  for (const bd of bdResult.rows) {
    bd.floors = flResult.rows.filter((fl) => fl.building_no === bd.no);
  }
  return bdResult.rows;
}

export async function queryMap(building: number, floor: number) {
  const result1 = await pg.query(
    "SELECT * FROM floors WHERE building_no = $1 and no = $2",
    [building, floor]
  );
  if (result1.rows.length === 0) {
    return null;
  }
  const result2 = await pg.query("SELECT * FROM buildings WHERE no = $1", [
    building,
  ]);
  return {
    floor: result1.rows[0],
    building: result2.rows[0],
  };
}

export async function updateMap(args: {
  building: number;
  floor: number;
  points: any;
  lines: any;
  paths: any;
}) {
  const result = await pg.query(
    "UPDATE floors SET points = $1, lines = $2, paths = $3 WHERE building_no = $4 and no = $5",
    [args.points, args.lines, args.paths, args.building, args.floor]
  );
  return result;
}
