import { Request, Response } from "express";
import { queryDevices, insertDevice, updateDevice } from "../models/device";

export async function getDevices(req: Request, res: Response) {
  try {
    const devices = await queryDevices();
    res.json({ status: 200, data: devices });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}

export async function createDevice(req: Request, res: Response) {
  try {
    const id = await insertDevice(req.body);
    res.json({ status: 200, data: id });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}

export async function editDevice(req: Request, res: Response) {
  try {
    await updateDevice(req.body);
    res.json({ status: 200, statusText: "设备更新成功" });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}
