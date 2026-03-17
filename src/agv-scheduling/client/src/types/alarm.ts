export interface AlarmRecord {
  /** 编号 */
  id: number;
  /** agv编号 */
  agv_id: number;
  /** agv名称 */
  agv_name: string;
  /** 检查状态: 1.检查中 2.检查通过 3.检查失败 */
  tr_check_state: number;
  /** 检查错误: 0.正常 1.分步衔接错误 2.里程错误 */
  tr_check_error: number;
  /** 错误码 */
  err1: number;
  /** 错误码 */
  err2: number;
  /** 错误码 */
  err3: number;
  /** 错误码 */
  err4: number;
  /** 错误码 */
  err5: number;
  /** 错误码 */
  warnings: number;
  /** 执行代码 */
  code: string;
  /** 任务解析 */
  body: string;
  /** 创建日期 */
  created_at: string;
}
