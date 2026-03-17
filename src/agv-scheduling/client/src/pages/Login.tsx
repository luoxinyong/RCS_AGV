import { useNavigate } from "react-router-dom";
import { useEffect, useState } from "react";
import { App, Button, Checkbox, Input, Space } from "antd";
import { userApi } from "@/network/api";
import { localStorageKey } from "@/config";
import Logo from "@/assets/images/logo.png";
import LoginBackground from "@/assets/images/login_bg.png";

export default function Login() {
  const navigate = useNavigate();
  const { notification } = App.useApp();

  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [rememberMe, setRememberMe] = useState(false);

  const onClickRememberMe = async (checked: boolean) => {
    setRememberMe(checked);
    localStorage.setItem(localStorageKey.rememberMe, checked ? "1" : "0");
  };

  const onClickLogin = async () => {
    const result = await userApi.login(username, password);
    if (result.status === 200) {
      // 1.保存账号
      localStorage.setItem(localStorageKey.username, username);
      // 2.保存密码
      if (rememberMe) {
        localStorage.setItem(localStorageKey.password, btoa(password));
      } else {
        localStorage.removeItem(localStorageKey.password);
      }
      // 3.保存token
      localStorage.setItem(localStorageKey.token, result.data.token);
      // 4.保存权限
      localStorage.setItem(localStorageKey.permission, result.data.permission);
      navigate("/", { replace: true });
    } else {
      notification.error({ message: result.statusText });
    }
  };

  useEffect(() => {
    // 本地查询账号
    const username = localStorage.getItem(localStorageKey.username);
    if (username) {
      setUsername(username);
    }
    // 本地查询记住密码
    const rememberMe = localStorage.getItem(localStorageKey.rememberMe) === "1";
    setRememberMe(rememberMe);
    // 本地查询密码
    if (rememberMe) {
      const password = localStorage.getItem(localStorageKey.password) ?? "";
      setPassword(atob(password));
    }
  }, []);

  return (
    <div className="h-screen w-screen">
      <div className="fixed inset-0 flex items-center justify-center z-50">
        <div className="bg-white rounded-lg shadow-lg w-[390px] p-8 mb-24">
          <Space size="middle">
            <img src={Logo} alt="Logo" className="w-8 h-7" />
            <label className="text-xl font-bold">AGV调度系统</label>
          </Space>
          <div className="mt-6">
            <label className="block text-gray-700 text-sm font-semibold mb-2">
              用户名:
            </label>
            <Input
              type="text"
              size="large"
              placeholder="请输入工号"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
            />
          </div>
          <div className="mt-6">
            <label className="block text-gray-700 text-sm font-semibold mb-2">
              密码:
            </label>
            <Input
              type="password"
              size="large"
              placeholder="请输入密码"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
            />
          </div>
          <Checkbox
            className="my-6"
            checked={rememberMe}
            onChange={(e) => onClickRememberMe(e.target.checked)}
          >
            记住密码
          </Checkbox>
          <Button type="primary" block size="large" onClick={onClickLogin}>
            登录
          </Button>
        </div>
      </div>
      <div className="absolute inset-0 overflow-hidden bg-white flex flex-col">
        <div className="flex-1 bg-gradient-to-b from-[#4bbb9c] to-[white]"></div>
        <img src={LoginBackground} alt="" className="h-1/2 pb-8 mx-auto" />
      </div>
    </div>
  );
}
