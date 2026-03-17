import dayjs from "dayjs";
import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { Button } from "antd/lib";
import { LogOut } from "lucide-react";
import { localStorageKey } from "@/config";

export default function Header() {
  const now = dayjs();
  const [currentDate, setCurrentDate] = useState(now.format("YYYY-MM-DD"));
  const [currentTime, setCurrentTime] = useState(now.format("HH:mm:ss"));
  const [currentDay, setCurrentDay] = useState(now.format("dddd"));

  useEffect(() => {
    const timer = setInterval(() => {
      const now = dayjs();
      setCurrentDate(now.format("YYYY-MM-DD"));
      setCurrentTime(now.format("HH:mm:ss"));
      setCurrentDay(now.format("dddd"));
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  const navigate = useNavigate();
  const onLogout = () => {
    localStorage.removeItem(localStorageKey.token);
    localStorage.removeItem(localStorageKey.permission);
    navigate("/login", { replace: true });
  };

  return (
    <div className="bg-white flex items-center px-5 py-2 shadow-sm relative">
      <h1 className="text-xl font-semibold text-gray-800 absolute left-1/2 transform -translate-x-1/2">
        AGV 机器人调度系统
      </h1>

      <div className="mr-auto">
        <Button icon={<LogOut className="w-4 pt-1" />} onClick={onLogout}>
          退出登录
        </Button>
      </div>

      <div className="ml-auto flex items-center justify-between">
        <span className="text-gray-600">{currentTime}</span>
        <div className="border-l border-gray-400 h-4 mx-3" />
        <div className="flex flex-col space-y-0.5 items-center">
          <span className="text-sm text-gray-600">{currentDay}</span>
          <span className="text-xs text-gray-600">{currentDate}</span>
        </div>
      </div>
    </div>
  );
}
