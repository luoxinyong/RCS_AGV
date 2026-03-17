import type { ArcRoute, LineRoute, Point } from "./map-editor";

export interface Floor {
  /** 楼层号 */
  no: number;
  /** 描述 */
  description: string;
  /** agv原点 */
  agv_origin: { x: number; y: number };
  /** 取放点 */
  points?: Point[];
  /** 直线路径 */
  lines?: LineRoute[];
  /** 弧线路径 */
  paths?: ArcRoute[];
}

export interface Building {
  /** 楼栋号 */
  no: number;
  /** 楼栋宽度 */
  width: number;
  /** 楼栋长度 */
  // height: number;
  /** 建筑原点（左上角） */
  origin: { x: number; y: number };
  /** 楼层 */
  floors: Floor[];
}
