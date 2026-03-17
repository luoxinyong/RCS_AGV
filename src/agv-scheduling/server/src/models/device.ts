import { pg } from "../services/postgres";
import type { AGVDevice } from "../types/device";

export async function queryDevices(): Promise<AGVDevice[]> {
  const result = await pg.query(
    "SELECT * FROM devices WHERE enabled = true ORDER BY id"
  );

  const devices = result.rows;
  // if (devices.length > 0) {
  //   // 查询每个设备的最后一条记录的时间戳
  //   const queries = devices.map((d) => {
  //     const sTableName = getSTableNameFromType(d.type);
  //     return `select last(ts) as ts, device_id as id from iot.${sTableName} where device_id = '${d.id}'`;
  //   });
  //   const tdSql = queries.join(" union all ");
  //   const tdResult = await executeTDEngineQuery(tdSql);
  //   // 遍历设备列表，设置每个设备的lastTs字段
  //   for (const d of devices) {
  //     d.lastTs = tdResult.find((r) => r.id == d.id)?.ts;
  //   }
  // }
  return devices;
}

export async function insertDevice(device: AGVDevice) {
  const result = await pg.query(
    "INSERT INTO devices (name, ip, port, remark, enabled) " +
      "VALUES ($1, $2, $3, $4, $5) " +
      "RETURNING id",
    [device.name, device.ip, device.port, device.remark, true]
  );
  if (result.rows.length > 0) {
    return result.rows[0].id;
  } else {
    throw new Error("插入设备失败");
  }
}

export async function updateDevice(device: AGVDevice) {
  const result = await pg.query(
    "UPDATE devices SET name = $1, ip = $2, port = $3, remark = $4 WHERE id = $5",
    [device.name, device.ip, device.port, device.remark, device.id]
  );
  return result;
}
