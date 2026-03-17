import { queryDoors } from "../models/door";
import type { Door } from "../types/door";

export default class DoorService {
  private static instance: DoorService | null = null;
  private timer?: NodeJS.Timeout;
  private doors: Door[] = [];

  public static getInstance(): DoorService {
    if (!DoorService.instance) {
      DoorService.instance = new DoorService();
    }
    return DoorService.instance;
  }

  private constructor() {}

  /** 立即刷新一次，然后每隔 interval 刷新 */
  public start(interval: number = 60_000) {
    this.stop();
    this.refresh();
    this.timer = setInterval(this.refresh, interval);
  }

  /** 关闭定时刷新服务 */
  public stop() {
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = undefined;
    }
  }

  private async refresh() {
    try {
      const start = new Date();
      console.log("更新卷帘门:", start.toLocaleString());
      this.doors = await queryDoors();
      console.log(
        "更新完成:",
        `${this.doors.length}扇门, ${Date.now() - start.getTime()}ms`
      );
    } catch (err) {
      console.error("❌ refresh error:", err);
    }
  }

  public getDoors() {
    return this.doors.map((e) => e);
  }

  public getDoorFromCargoSize(cargoSize?: number) {
    if (!cargoSize) return undefined;
    // 转成16位二进制字符串
    const binaryString = cargoSize.toString(2).padStart(16, "0");
    // 取高3-8位，门的最大编号 id <= 63
    const high = binaryString.slice(2, 8);
    // 转成十进制
    const doorId = parseInt(high, 2);
    // 根据 doorId 查找对应的门
    return this.doors.find((d) => d.id === doorId);
  }
}
