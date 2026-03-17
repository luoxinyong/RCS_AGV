import { Request, Response } from "express";
import { queryAlarms } from "../models/alarms";

export async function getAlarms(req: Request, res: Response) {
  try {
    const { page, pageSize, sorterField, order } = req.query;
    const o = String(order).toLowerCase();
    const devices = await queryAlarms({
      page: Number(page),
      pageSize: Number(pageSize),
      sorterField: sorterField && String(sorterField),
      ascend: o === "asc" || o === "ascend",
    });
    res.json({ status: 200, data: devices });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}
