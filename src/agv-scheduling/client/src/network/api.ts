import http from "./axios";
import type { Pagination } from "@/types/pagination";
import type { UserInfo } from "@/types/user";
import type { AGVDevice } from "@/types/device";
import type { AlarmRecord } from "@/types/alarm";
import type { TaskRecord } from "@/types/task";
import type { Building } from "@/types/building";
import type { Door } from "@/types/door";

export const userApi = {
  login: (username: string, password: string) =>
    http.post<UserInfo>("api/v1/user/login", { username, password }),
};

export const deviceApi = {
  getDevices: () => http.get<AGVDevice[]>("/api/v1/device"),
  addDevice: (data: AGVDevice) => http.post("/api/v1/device/add", data),
  editDevice: (data: AGVDevice) => http.post("/api/v1/device/edit", data),
};

export const alarmApi = {
  getAlarms: (params: {
    page: number;
    pageSize: number;
    sorterField?: string;
    order?: string;
  }) => http.get<Pagination<AlarmRecord>>("/api/v1/alarm", { params }),
};

export const taskApi = {
  getTaskLogs: (params: {
    page: number;
    pageSize: number;
    typeFilters: string;
    sorterField?: string;
    order?: string;
  }) => http.get<Pagination<TaskRecord>>("/api/v1/task/logs", { params }),
};

export const mapApi = {
  getMaps: () => http.get<Building[]>("/api/v1/map/all"),
  getMap: (params: { building: string; floor: string }) =>
    http.get("/api/v1/map/find", { params }),
  editMap: (data: {
    building: string;
    floor: string;
    points: string;
    lines: string;
    paths: string;
  }) => http.post("/api/v1/map/edit", data),
};

export const doorApi = {
  getDoors: () => http.get<Door[]>("/api/v1/doors"),
};
