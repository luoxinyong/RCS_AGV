import dayjs from "dayjs";
import { useState, useEffect } from "react";
import { Battery, Wifi, WifiOff } from "lucide-react";
import { servers } from "@/config";
import { deviceApi } from "@/network/api";
import useApi from "@/hooks/useApi";
import useWebSocket from "@/hooks/useWebSocket";
import type { AGVDevice } from "@/types/device";
import type { AGVStatus } from "@/types/map-editor";

export default function MonitorStatus() {
  const { execute, data: devices } = useApi<AGVDevice[]>();

  const [lastStatus, setLastStatus] = useState<AGVStatus[]>([]);

  useWebSocket(servers.websocketServer, {
    onMessage(message) {
      if (message?.type === "update") {
        setLastStatus(message?.data);
      } else {
        // console.log(new Date().toLocaleString("zh-CN"), { message });
      }
    },
  });

  useEffect(() => {
    execute(deviceApi.getDevices);
  }, [execute]);

  return (
    <div className="p-4 space-6">
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {devices?.map((e) => {
          const deviceStatus = lastStatus.find((d) => d.id === e.id.toString());
          return (
            <div
              key={e.id}
              className="bg-white rounded-lg shadow-lg p-6 border"
            >
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold text-gray-800">
                  {e.name}
                </h3>
                <div className="flex items-center space-x-2">
                  {deviceStatus?.isOnline ? (
                    <Wifi className="w-5 h-5 text-green-500" />
                  ) : (
                    <WifiOff className="w-5 h-5 text-red-500" />
                  )}
                  <span
                    className={`px-2 py-1 rounded-full text-xs font-medium ${
                      deviceStatus?.isOnline
                        ? "bg-green-100 text-green-800"
                        : "bg-gray-100 text-gray-800"
                    }`}
                  >
                    {deviceStatus?.isOnline ? "在线" : "离线"}
                  </span>
                </div>
              </div>

              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">IP地址:</span>
                  <span className="text-sm font-medium">
                    {e.ip}:{e.port}
                  </span>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">位置:</span>
                  <span className="text-sm font-medium">
                    x:{" "}
                    {deviceStatus?.position?.x
                      ? Number((deviceStatus?.position?.x / 1000).toFixed(2)) +
                        " m"
                      : "--"}
                    , y:{" "}
                    {deviceStatus?.position?.y
                      ? Number((deviceStatus?.position?.y / 1000).toFixed(2)) +
                        " m"
                      : "--"}
                  </span>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">电池电量:</span>
                  <div className="flex items-center space-x-2">
                    <Battery
                      className={`w-4 h-4 ${
                        (deviceStatus?.battery ?? 0) > 30
                          ? "text-green-500"
                          : (deviceStatus?.battery ?? 0) > 20
                          ? "text-yellow-500"
                          : "text-red-500"
                      }`}
                    />
                    <span className="text-sm font-medium">
                      {deviceStatus?.battery
                        ? `${deviceStatus?.battery} %`
                        : "--"}
                    </span>
                  </div>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">速度:</span>
                  <span className="text-sm font-medium">
                    {deviceStatus?.speed
                      ? Number((deviceStatus?.speed / 1000).toFixed(2)) + " m/s"
                      : "--"}
                  </span>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">角度:</span>
                  <div className="flex items-center space-x-2">
                    <span className="text-sm font-medium">
                      {deviceStatus?.angle ? `${deviceStatus?.angle}°` : "--"}
                    </span>
                  </div>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">货叉高度:</span>
                  <div className="flex items-center space-x-2">
                    <span className="text-sm font-medium">
                      {deviceStatus?.liftHeight
                        ? `${deviceStatus?.liftHeight}mm`
                        : "--"}
                    </span>
                  </div>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">当前时间:</span>
                  <div className="flex items-center space-x-2">
                    <span className="text-sm font-medium">
                      {deviceStatus?.timestamp
                        ? dayjs(deviceStatus.timestamp).format(
                            "YYYY-MM-DD HH:mm:ss"
                          )
                        : "--"}
                    </span>
                  </div>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
