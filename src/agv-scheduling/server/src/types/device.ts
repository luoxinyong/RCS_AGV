export interface AGVDevice {
  /** AGV编号 */
  id: number;
  /** 名称 */
  name: string;
  /** ip地址 */
  ip: string;
  /** 端口号 */
  port: number;
  /** 备注 */
  remark?: string;
}

export interface AGVStatus {
  /** 当前处理命令 */
  tr_cmd: number;
  /** 该小时内的第n个任务 */
  tr_id_index: number;
  /** 当前时 */
  tr_id_hour: number;
  /** 当前日 */
  tr_id_day: number;
  /** 当前月 */
  tr_id_month: number;
  /** 当前年 */
  tr_id_year: number;
  /** 发送者ip后缀段 */
  tr_sender: number;
  /** 任务类型 */
  tr_type: number;
  /** 任务步骤总数 */
  tr_step_size: number;
  /** 任务启动延迟 */
  tr_start_delay: number;
  /** 1.检查中 2.检查通过 3.检查失败 */
  tr_check_state: number;
  /** 0.无错 1.分布衔接错误 2.里程错误 */
  tr_check_error: number;
  /** 高8位: 卷帘门id，低8位: 避障区域 (废弃: 任务总里程，单位精度0.1米.) */
  tr_odo_total: number;
  /** 任务已执行时间，单位秒 */
  tr_execute_time: number;
  /** 任务完成百分比，0~100 */
  tr_cplt_percent: number;
  /** 任务当前步骤号 */
  tr_step_index: number;
  /** 任务暂停原因信息码，暂不用 */
  tr_pause_info: number;
  /** 当前分布完成百分比 */
  step_cplt_percent: number;
  /** 当前分步类型 */
  step_type: number;
  /** 下一分步类型 */
  step_type_next: number;
  /** 是否处于暂停：0.无 1.暂停 */
  step_en_pause: number;
  /** 是否在当前步骤结束时停车，0.不停 1.停 */
  step_is_end_stop: number;
  /** 高8位: 卷帘门id，低8位: 避障区域 */
  // cargo_size: number;
  /** 分步起点 */
  id_a: number;
  /** 分步终点 */
  id_b: number;

  /** 开机时长 */
  ts: number;
  /** 当前x坐标 */
  px: number;
  /** 当前y坐标 */
  py: number;
  /** 车头方向，取值0~3600 */
  pa: number;
  /** 货叉高度 */
  lift_height: number;
  /** 举升高度 */
  lift_speed: number;
  /** 当前速度 */
  traction_speed: number;
  /** 是否驻车：0.运动中 1.已驻车 */
  traction_is_parked: number;
  /** 当前舵角 */
  steer_angle: number;
  /** 电池电量，取值0~100 */
  energy_level: number;
  /** 充电状态：0.未知 1.充电中 2.停止充电 */
  status_charge: number;
  /** 通信质量 */
  radio_level: number;
  /** 定位质量 */
  nav_level: number;
  /** 即 task_state，显示任务状态机 */
  lv1_tr_sate: number;
  /** 显示模式状态机 */
  lv2_mode_state: number;
  /** 显示自动状态机 */
  lv3_auto_state: number;
  /** 硬件版本信息码 */
  version_hw: number;
  /** 软件版本信息码 */
  version_sw: number;
  /** 错误码，非0表示有错 */
  err1: number;
  /** 错误码，非0表示有错 */
  err2: number;
  /** 错误码，非0表示有错 */
  err3: number;
  /** 错误码，非0表示有错 */
  err4: number;
  /** 错误码，非0表示有错 */
  err5: number;
  /** 错误码，非0表示有错 */
  warnings: number;
  /** 是否有货物在车上，1.有 0.没有 */
  cargo_loaded: number;

  /** 是否已发送开门指令 */
  openCommandSended?: boolean;
}
