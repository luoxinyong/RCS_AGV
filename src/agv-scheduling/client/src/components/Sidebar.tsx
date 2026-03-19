import { useState } from "react";
import { Link, useLocation, useNavigate } from "react-router-dom";
import { RobotOutlined } from "@ant-design/icons";
import {
  ChevronRight,
  SunMedium,
  // Settings,
  MonitorDot,
  MapIcon,
  TableIcon,
} from "lucide-react";

interface NavigationProps {
  name: string;
  path: string;
  icon?: React.FC;
  children?: NavigationProps[];
}

const navigation: NavigationProps[] = [
  { name: "总览", path: "/", icon: () => <SunMedium className="w-5 h-5" /> },
  // {
  //   name: "系统设置",
  //   path: "/settings",
  //   icon: () => <Settings className="w-5 h-5" />,
  // },
];

const navigationTree: NavigationProps[] = [
  {
    name: "设备监控",
    path: "/monitor",
    icon: () => <MonitorDot className="w-5 h-5" />,
    children: [{ name: "AGV叉车", path: "/monitor/status" }],
  },
  {
    name: "地图管理",
    path: "/map",
    icon: () => <MapIcon className="w-5 h-5" />,
    children: [
      { name: "车间地图", path: "/map/list" },
      { name: "路线规划", path: "/map/editor" },
    ],
  },
  {
    name: "调度日志",
    path: "/history",
    icon: () => <TableIcon className="w-5 h-5" />,
    children: [
      { name: "任务日志", path: "/history/task" },
      { name: "报警监控", path: "/history/alarm" },
    ],
  },
  {
  name: "智能交互",
  path: "/ai",
  icon: () => <RobotOutlined className="w-5 h-5" />,   // 需要 import
  children: [{ name: "任务助手", path: "/ai/chat" }],
},
];

export default function Sidebar() {
  const location = useLocation();

  return (
    <div className="h-full p-4 bg-white shadow-lg space-y-4 flex flex-col">
      <nav className="space-y-2">
        {navigation.map((item) => (
          <Link
            key={item.path}
            to={item.path}
            className={`flex items-center space-x-3 p-3 rounded-lg transition-colors ${
              location.pathname === item.path
                ? "bg-blue-50 text-blue-600"
                : "hover:bg-gray-100 text-gray-700"
            }`}
          >
            {item.icon?.({})}
            <span className="text-sm font-medium">{item.name}</span>
          </Link>
        ))}
      </nav>

      <div className="border-t" />

      <div className="flex-1 overflow-auto">
        <h3 className="text-sm font-medium text-gray-500 p-3">RCS</h3>
        <nav className="space-y-1">
          {navigationTree.map((e) => (
            <TreeItem key={e.path} item={e} />
          ))}
        </nav>
      </div>
    </div>
  );
}

function TreeItem({
  item,
  level = 0,
}: {
  item: NavigationProps;
  level?: number;
}) {
  const [isExpanded, setIsExpanded] = useState(false);

  const location = useLocation();
  const navigate = useNavigate();

  return (
    <div className="select-none">
      <div
        className={`cursor-pointer flex items-center space-x-2 px-2 rounded-lg transition-colors ${
          level > 0 ? "ml-6" : ""
        } ${
          location.pathname === item.path
            ? "bg-blue-50 text-blue-600"
            : "hover:bg-gray-100 text-gray-700"
        }`}
        onClick={() =>
          level === 0 ? setIsExpanded(!isExpanded) : navigate(item.path)
        }
      >
        {item.children && item.children.length > 0 && (
          <span
            className={`text-gray-400 transition-transform duration-200 ${
              isExpanded ? "rotate-90" : "rotate-0"
            }`}
          >
            <ChevronRight className="w-4 h-4" />
          </span>
        )}
        <div className="flex items-center flex-1 py-2">
          {item.icon?.({})}
          <span className="ml-2 text-sm">{item.name}</span>
        </div>
      </div>

      <div
        className={`overflow-hidden transition-all duration-300 ease-in-out ${
          isExpanded ? "max-h-screen" : "max-h-0"
        }`}
      >
        {item.children?.map((e) => (
          <div
            key={e.path}
            className={`mt-1 transition-all duration-100 ${
              isExpanded
                ? "translate-x-0 opacity-100"
                : "-translate-x-2 opacity-0"
            }`}
            style={{
              transitionDelay: isExpanded ? "100ms" : "0ms",
            }}
          >
            <TreeItem item={e} level={level + 1} />
          </div>
        ))}
      </div>
    </div>
  );
}
