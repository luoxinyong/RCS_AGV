import { AGVStatus } from "../types/device";

/** 任务类型 */
export enum TaskType {
  pause = 1, // 暂停
  execute, // 执行
  stop, // 停止
  send, // 发送
  active, // 解除原地等待
}

/** TaskType=send时，下发的分步任务 */
export interface SubTask {
  offset1?: number;
  offset2?: number;
  offset3?: number;
  offset4?: number;
  offset5?: number;
  offset6?: number;
  offset7?: number;
  offset8?: number;
  offset9?: number;
  offset10?: number;
  offset11?: number;
  offset12?: number;
  offset13?: number;
  offset14?: number;
  offset15?: number;
  offset16?: number;
  offset17?: number;
  offset18?: number;
  offset19?: number;
  offset20?: number;
}

/** 转换为任务发送指令 */
export function covertToTaskBuffer(taskType: TaskType, steps?: SubTask[]) {
  if (taskType != TaskType.send) {
    const buffer = Buffer.alloc(2);
    buffer.writeInt16BE(taskType, 0);
    return buffer;
  }

  const stepsLength = steps?.length ?? 0;

  const buffer = Buffer.alloc(20 + 40 * stepsLength);
  buffer.writeInt16BE(taskType, 0); // 命令
  buffer.writeInt16BE(1, 2); // TODO: 该小时内的n号任务
  const now = new Date();
  buffer.writeUInt16BE(now.getHours(), 4); // 当前小时
  buffer.writeUInt16BE(now.getDate(), 6); // 当前日
  buffer.writeUInt16BE(now.getMonth() + 1, 8); // 当前月
  buffer.writeUInt16BE(now.getFullYear(), 10); // 当前年
  // const ipSuffix = getLocalIP().split(".").map(Number)[3];
  buffer.writeInt16BE(30, 12); // TODO: ip后缀
  buffer.writeInt16BE(taskType, 14); // 命令
  buffer.writeInt16BE(stepsLength, 16); // 任务分布数量
  buffer.writeInt16BE(1, 18); // 延时启动秒数

  for (let i = 0; i < stepsLength; i++) {
    const step = steps?.[i];
    const index = 20 + 40 * i;
    buffer.writeInt16BE(step?.offset1 ?? 0, index);
    buffer.writeInt16BE(step?.offset2 ?? 0, index + 2);
    buffer.writeInt16BE(step?.offset3 ?? 0, index + 4);
    buffer.writeUInt16BE(step?.offset4 ?? 0, index + 6); // 无符号16位整形: 行驶距离可能大于32767
    buffer.writeUInt16BE(step?.offset5 ?? 0, index + 8); // 无符号16位整形: cargo_size
    buffer.writeInt16BE(step?.offset6 ?? 0, index + 10);
    buffer.writeInt16BE(step?.offset7 ?? 0, index + 12);
    buffer.writeInt16BE(step?.offset8 ?? 0, index + 14);
    buffer.writeInt16BE(step?.offset9 ?? 0, index + 16);
    buffer.writeInt16BE(step?.offset10 ?? 0, index + 18);
    buffer.writeInt16BE(step?.offset11 ?? 0, index + 20);
    buffer.writeInt16BE(step?.offset12 ?? 0, index + 22);
    buffer.writeInt16BE(step?.offset13 ?? 0, index + 24);
    buffer.writeInt16BE(step?.offset14 ?? 0, index + 26);
    buffer.writeInt16BE(step?.offset15 ?? 0, index + 28);
    buffer.writeInt16BE(step?.offset16 ?? 0, index + 30);
    buffer.writeInt16BE(step?.offset17 ?? 0, index + 32);
    buffer.writeInt16BE(step?.offset18 ?? 0, index + 34);
    buffer.writeInt16BE(step?.offset19 ?? 0, index + 36);
    buffer.writeInt16BE(step?.offset20 ?? 0, index + 38);
  }
  return buffer;
}

/** 调试: 解析任务发送指令 */
export function parseTaskHexString(hex: string) {
  const buffer = Buffer.from(hex.replace(/ /g, ""), "hex");
  const stepsLength = buffer.readInt16BE(16);
  const steps = [];
  for (let i = 0; i < stepsLength; i++) {
    const index = 20 + 40 * i;
    const step_type = buffer.readInt16BE(index);
    if (step_type == 1) {
      steps.push({
        step_type: "1.直线",
        vlimit: buffer.readUInt16BE(index + 2),
        heading: buffer.readInt16BE(index + 4),
        length: buffer.readUInt16BE(index + 6),
        cargo_size: buffer.readUInt16BE(index + 8),
        ax: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 10),
          buffer.readUInt16BE(index + 12)
        ),
        ay: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 14),
          buffer.readUInt16BE(index + 16)
        ),
        bx: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 18),
          buffer.readUInt16BE(index + 20)
        ),
        by: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 22),
          buffer.readUInt16BE(index + 24)
        ),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
        id_b: buffer.readInt16BE(index + 38),
      });
    } else if (step_type == 2) {
      steps.push({
        step_type: "2.弯道",
        vlimit: buffer.readUInt16BE(index + 2),
        heading: buffer.readInt16BE(index + 4),
        length: buffer.readUInt16BE(index + 6),
        cargo_size: buffer.readUInt16BE(index + 8),
        ax: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 10),
          buffer.readUInt16BE(index + 12)
        ),
        ay: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 14),
          buffer.readUInt16BE(index + 16)
        ),
        bx: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 18),
          buffer.readUInt16BE(index + 20)
        ),
        by: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 22),
          buffer.readUInt16BE(index + 24)
        ),
        cx: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 26),
          buffer.readUInt16BE(index + 28)
        ),
        cy: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 30),
          buffer.readUInt16BE(index + 32)
        ),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
        id_b: buffer.readInt16BE(index + 38),
      });
    } else if (step_type == 3) {
      steps.push({
        step_type: "3.举升",
        height: buffer.readUInt16BE(index + 2),
        hspeed: buffer.readUInt16BE(index + 4),
        stop_before_done: buffer.readInt16BE(index + 6),
        done_wait_time: buffer.readInt16BE(index + 8),
        cargo_size: buffer.readUInt16BE(index + 10),
        is_where: buffer.readInt16BE(index + 12),
        heading: buffer.readInt16BE(index + 14),
        ax: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 16),
          buffer.readUInt16BE(index + 18)
        ),
        ay: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 20),
          buffer.readUInt16BE(index + 22)
        ),
        position2: buffer.readInt16BE(index + 24),
        position3: buffer.readInt16BE(index + 26),
        disc_angle: buffer.readInt16BE(index + 28),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
      });
    } else if (step_type == 4) {
      steps.push({
        step_type: "4.充电",
        charge_mode: buffer.readInt16BE(index + 2),
        charge_limit: buffer.readInt16BE(index + 4),
        cargo_size: buffer.readUInt16BE(index + 6),
        heading: buffer.readInt16BE(index + 8),
        ax: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 10),
          buffer.readUInt16BE(index + 12)
        ),
        ay: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 14),
          buffer.readUInt16BE(index + 16)
        ),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
      });
    } else if (step_type == 9) {
      steps.push({
        step_type: "9.自调节",
        vlimit: buffer.readUInt16BE(index + 2),
        heading: buffer.readInt16BE(index + 4),
        length: buffer.readUInt16BE(index + 6),
        cargo_size: buffer.readUInt16BE(index + 8),
        ax: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 10),
          buffer.readUInt16BE(index + 12)
        ),
        ay: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 14),
          buffer.readUInt16BE(index + 16)
        ),
        bx: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 18),
          buffer.readUInt16BE(index + 20)
        ),
        by: joinUInt16sToInt32(
          buffer.readUInt16BE(index + 22),
          buffer.readUInt16BE(index + 24)
        ),
        depth: buffer.readUInt16BE(index + 26),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
        id_b: buffer.readInt16BE(index + 38),
      });
    } else if (step_type == 99) {
      steps.push({
        step_type: "99.结束任务",
        task_id_index: buffer.readUInt16BE(index + 32),
        map_id: buffer.readInt16BE(index + 34),
        id_a: buffer.readInt16BE(index + 36),
        id_b: buffer.readInt16BE(index + 38),
      });
    }
  }
  return {
    tr_cmd: buffer.readInt16BE(0),
    tr_id_index: buffer.readInt16BE(2),
    tr_id_hour: buffer.readUInt16BE(4),
    tr_id_day: buffer.readUInt16BE(6),
    tr_id_month: buffer.readUInt16BE(8),
    tr_id_year: buffer.readUInt16BE(10),
    tr_sender: buffer.readInt16BE(12),
    tr_type: buffer.readInt16BE(14),
    tr_step_size: stepsLength,
    tr_start_delay: buffer.readInt16BE(18),
    steps,
  };
}

/** 解析状态响应 */
export function parseStatusBuffer(data: Buffer | String): AGVStatus {
  const buffer = Buffer.isBuffer(data)
    ? data
    : Buffer.from(data.replace(/ /g, ""), "hex");
  return {
    tr_cmd: buffer.readInt16BE(0),
    tr_id_index: buffer.readInt16BE(2),
    tr_id_hour: buffer.readUInt16BE(4),
    tr_id_day: buffer.readUInt16BE(6),
    tr_id_month: buffer.readUInt16BE(8),
    tr_id_year: buffer.readUInt16BE(10),
    tr_sender: buffer.readInt16BE(12),
    tr_type: buffer.readInt16BE(14),
    tr_step_size: buffer.readInt16BE(16),
    tr_start_delay: buffer.readInt16BE(18),
    tr_check_state: buffer.readInt16BE(20),
    tr_check_error: buffer.readInt16BE(22),
    tr_odo_total: buffer.readInt16BE(24),
    tr_execute_time: buffer.readInt16BE(26),
    tr_cplt_percent: buffer.readInt16BE(28),
    tr_step_index: buffer.readInt16BE(30),
    tr_pause_info: buffer.readInt16BE(32),
    step_cplt_percent: buffer.readInt16BE(34),
    step_type: buffer.readInt16BE(36),
    step_type_next: buffer.readInt16BE(38),
    step_en_pause: buffer.readInt16BE(40),
    step_is_end_stop: buffer.readInt16BE(42),
    // cargo_size: buffer.readUInt16BE(52),
    id_a: buffer.readInt16BE(80),
    id_b: buffer.readInt16BE(82),
    ts: buffer.readUInt32BE(84),
    px: joinUInt16sToInt32(buffer.readUInt16BE(88), buffer.readUInt16BE(90)),
    py: joinUInt16sToInt32(buffer.readUInt16BE(92), buffer.readUInt16BE(94)),
    pa: buffer.readInt16BE(96) / 10,
    lift_height: buffer.readInt16BE(98),
    lift_speed: buffer.readInt16BE(100),
    traction_speed: buffer.readInt16BE(102),
    traction_is_parked: buffer.readInt16BE(104),
    steer_angle: buffer.readInt16BE(106),
    energy_level: buffer.readInt16BE(108),
    status_charge: buffer.readInt16BE(110),
    radio_level: buffer.readInt16BE(112),
    nav_level: buffer.readInt16BE(114),
    lv1_tr_sate: buffer.readInt16BE(116),
    lv2_mode_state: buffer.readInt16BE(118),
    lv3_auto_state: buffer.readInt16BE(120),
    version_hw: buffer.readUInt16BE(122),
    version_sw: buffer.readUInt16BE(124),
    err1: buffer.readUInt16BE(126),
    err2: buffer.readUInt16BE(128),
    err3: buffer.readUInt16BE(130),
    err4: buffer.readUInt16BE(132),
    err5: buffer.readUInt16BE(134),
    warnings: buffer.readUInt16BE(136),
    cargo_loaded: buffer.readUInt16BE(138),
  };
}

/** 将两个高低16位无符号整数转换成一个有符号32位整数。用于解析指令时的坐标转换 */
function joinUInt16sToInt32(low: number, high: number) {
  return (high << 16) | low;
}
