export interface TaskRecord {
  /** 编号 */
  id: number;
  /** agv编号 */
  agv_id: number;
  /** agv名称 */
  agv_name: string;
  /** 类型 */
  type: string;
  /** 耗时 */
  duration: number;
  /** 执行代码 */
  code: string;
  /** 任务解析 */
  body: string;
  /** 创建日期 */
  created_at: string;
}
