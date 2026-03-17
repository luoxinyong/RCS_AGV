import jwt from "jsonwebtoken";
import { Request, Response } from "express";
import { getUserInfo } from "../models/user";
import APP_CONFIG from "../config/app";

export async function login(req: Request, res: Response) {
  try {
    const { username, password } = req.body;
    const users = await getUserInfo(username, password);
    if (users.length === 0) {
      res.json({ status: 401, statusText: "用户名或密码错误" });
      return;
    }
    // 提取用户信息
    const userInfo = {
      employeeNo: users[0].employee_no,
      employeeName: users[0].employee_name,
      permission: users[0].permission,
    };
    // 生成Token
    const tokenStr = jwt.sign(userInfo, APP_CONFIG.JWT_SECRET, {
      expiresIn: "7 days", // token 有效期
    });
    res.json({
      status: 200,
      statusText: "登录成功",
      data: {
        ...userInfo,
        token: "Bearer " + tokenStr,
      },
    });
  } catch (err) {
    res.json({ status: 500, statusText: "登录失败" });
  }
}
