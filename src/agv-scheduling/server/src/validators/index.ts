import { z } from "zod";

export const loginSchema = z.object({
  username: z.string().min(1, "用户名不能为空"),
  password: z.string().min(1, "密码不能为空"),
});

export const createDeviceSchema = z.object({
  name: z
    .string({ required_error: "name不能为空" })
    .min(1, "设备名称不能为空")
    .trim(),
  ip: z
    .string({ required_error: "ip不能为空" })
    .min(1, "通信地址不能为空")
    .trim(),
  port: z
    .number({ required_error: "port不能为空" })
    .int("端口号必须为整数")
    .min(1, "端口号不能为空"),
  remark: z.string().max(200, "备注不能超过200字符").nullable(), // 可为null
});

export const editDeviceSchema = createDeviceSchema.extend({
  id: z.number({ required_error: "id不能为空" }).int().positive("必须为正整数"),
});

export const paginationSchema = z.object({
  page: z.string({ required_error: "page不能为空" }),
  pageSize: z.string({ required_error: "pageSize不能为空" }),
  // typeFilters: z.string().optional(),
  sorterField: z.string().optional(),
  order: z.string().optional(),
});

export const searchMapSchema = z.object({
  building: z.string({ required_error: "building不能为空" }),
  floor: z.string({ required_error: "floor不能为空" }),
});

export const editMapSchema = searchMapSchema.extend({
  points: z.string({ required_error: "points不能为空" }),
  lines: z.string({ required_error: "lines不能为空" }),
  paths: z.string({ required_error: "paths不能为空" }),
});
