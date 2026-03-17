export const servers = {
  apiServer: "http://172.17.80.25:3001",
  websocketServer: "ws://172.17.80.25:3001",
  // PG_HOST: 172.16.18.91  | localhost
  // PG_PASSWORD: hewr@4r43 | vvho123456
  // PG_DATABASE: postgres  | agv
};

export const localStorageKey = {
  // 登录页
  rememberMe: "isRememberMe",
  username: "username",
  password: "password",
  token: "token",
  permission: "permission",

  // 路径规划
  building: "building",
  floor: "floor",
};

export const agvKey = {
  /** 默认货叉举起高度 */
  raiseHeight: 300,
  /** 默认货叉放下高度 */
  lowerHeight: 90,
  /** 举升速度 */
  raiseSpeed: 300,
  /** 自旋速度 */
  spinSpeed: 300,
  /** 弯道速度 */
  curveSpeed: 400,
  /** 直线速度 */
  straightSpeed: 500,
  /** 角度范围 */
  angleRange: 35,
  /** 取货(自调节): 可识别距离 */
  recognizableDistance: 1800,
};
