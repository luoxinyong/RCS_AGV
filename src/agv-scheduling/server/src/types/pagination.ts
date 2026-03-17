export interface Pagination<T> {
  /** 编号 */
  data: T[];

  /** 总数 */
  total: number;

  /** 当前页码 */
  page: number;

  /** 分页数量 */
  pageSize: number;
}
