import { useState } from "react";
import {
  Cable,
  Wifi,
  WifiOff,
  BatteryLow,
  AlertTriangle,
  Clock,
} from "lucide-react";
import { servers } from "@/config";
import useWebSocket from "@/hooks/useWebSocket";
import type { AGVStatus } from "@/types/map-editor";

const Overview: React.FC = () => {
  const [deviceStatus, setDeviceStatus] = useState<AGVStatus[]>([]);

  // 实时获取设备状态
  useWebSocket(servers.websocketServer, {
    onMessage(message) {
      if (message?.type === "update") {
        setDeviceStatus(message?.data);
      }
    },
  });

  return (
    <div className="p-6 space-y-6">
      {/* 统计卡片 */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        <div className="bg-white rounded-lg shadow-md p-6 border-l-4 border-blue-500">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">总设备数</p>
              <p className="text-2xl font-bold text-gray-900">
                {deviceStatus.length}
              </p>
            </div>
            <div className="bg-blue-100 p-3 rounded-full">
              <Cable className="w-6 h-6 text-blue-600" />
            </div>
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6 border-l-4 border-green-500">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">在线设备</p>
              <p className="text-2xl font-bold text-gray-900">
                {deviceStatus.filter((e) => e.isOnline).length}
              </p>
            </div>
            <div className="bg-green-100 p-3 rounded-full">
              <Wifi className="w-6 h-6 text-green-600" />
            </div>
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6 border-l-4 border-red-500">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">离线设备</p>
              <p className="text-2xl font-bold text-gray-900">
                {deviceStatus.filter((e) => !e.isOnline).length}
              </p>
            </div>
            <div className="bg-red-100 p-3 rounded-full">
              <WifiOff className="w-6 h-6 text-red-600" />
            </div>
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6 border-l-4 border-yellow-500">
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm font-medium text-gray-600">低电量设备</p>
              <p className="text-2xl font-bold text-gray-900">
                {
                  deviceStatus.filter((e) => e.isOnline && e.battery < 20)
                    .length
                }
              </p>
            </div>
            <div className="bg-yellow-100 p-3 rounded-full">
              <BatteryLow className="w-6 h-6 text-yellow-600" />
            </div>
          </div>
        </div>
      </div>

      {/* 系统状态概览 */}
      <div className="bg-white rounded-lg shadow-md p-6">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">系统概览</h2>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="border rounded-lg p-4">
            <div className="flex items-center space-x-3 mb-2">
              <Clock className="w-5 h-5 text-blue-500" />
              <span className="font-medium text-gray-700">运行状态</span>
            </div>
            <p className="text-sm text-gray-600">系统运行正常</p>
          </div>

          <div className="border rounded-lg p-4">
            <div className="flex items-center space-x-3 mb-2">
              <AlertTriangle className="w-5 h-5 text-yellow-500" />
              <span className="font-medium text-gray-700">报警信息</span>
            </div>
            <p className="text-sm text-gray-600">当前无报警</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Overview;
