import { Request, Response } from "express";
import { queryDoors } from "../models/door";

export async function getDoors(req: Request, res: Response) {
  try {
    const doors = await queryDoors();
    res.json({ status: 200, data: doors });
  } catch (err: any) {
    res.json({ status: 500, statusText: err.message });
  }
}
