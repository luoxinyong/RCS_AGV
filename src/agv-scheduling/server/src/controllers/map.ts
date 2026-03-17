import { Request, Response } from "express";
import { queryMaps, queryMap, updateMap } from "../models/map";

export async function getMaps(req: Request, res: Response) {
  try {
    const maps = await queryMaps();
    res.json({ status: 200, data: maps });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}

export async function getMap(req: Request, res: Response) {
  try {
    const { building, floor } = req.query;
    const map = await queryMap(Number(building), Number(floor));
    if (map) {
      res.json({ status: 200, data: map });
    } else {
      res.json({ status: 500, statusText: "未查询到数据" });
    }
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}

export async function editMap(req: Request, res: Response) {
  try {
    await updateMap(req.body);
    res.json({ status: 200, statusText: "配置更新成功" });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}
