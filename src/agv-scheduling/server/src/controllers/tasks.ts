import { Request, Response } from "express";
import { queryTaskLogs } from "../models/tasks";

export async function getTaskLogs(req: Request, res: Response) {
  try {
    const { page, pageSize, typeFilters, sorterField, order } = req.query;
    const o = String(order).toLowerCase();
    const devices = await queryTaskLogs({
      page: Number(page),
      pageSize: Number(pageSize),
      typeFilters: String(typeFilters),
      sorterField: sorterField && String(sorterField),
      ascend: o === "asc" || o === "ascend",
    });
    res.json({ status: 200, data: devices });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}
