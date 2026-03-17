import { createBrowserRouter } from "react-router-dom";
import Layout from "@/layouts/Layout";
import Auth from "@/pages/Auth";
import Unknown from "@/pages/Unknown";
import Login from "@/pages/Login";
import Overview from "@/pages/Overview";
import Settings from "@/pages/Settings";
import MonitorStatus from "@/pages/monitor/Status";
import MapList from "@/pages/map/List";
import MapEditor from "@/pages/map/editor";
import HistoryTask from "@/pages/history/Task";
import HistoryAlarm from "@/pages/history/Alarm";

export const router = createBrowserRouter(
  [
    {
      path: "auth",
      element: <Auth />,
    },
    {
      path: "*",
      element: <Unknown />,
    },
    {
      path: "login",
      element: <Login />,
    },
    {
      path: "/",
      element: <Layout />,
      children: [
        {
          index: true,
          element: <Overview />,
        },
        {
          path: "settings",
          element: <Settings />,
        },
        {
          path: "monitor",
          children: [
            {
              path: "/monitor/status",
              element: <MonitorStatus />,
            },
          ],
        },
        {
          path: "map",
          children: [
            {
              path: "/map/list",
              element: <MapList />,
            },
            {
              path: "/map/editor",
              element: <MapEditor />,
            },
          ],
        },
        {
          path: "history",
          children: [
            {
              path: "/history/task",
              element: <HistoryTask />,
            },
            {
              path: "/history/alarm",
              element: <HistoryAlarm />,
            },
          ],
        },
      ],
    },
  ]
  // {
  //   patchRoutesOnNavigation: (opts) => {
  //     if (opts.path.startsWith("/meter/")) {
  //       return;
  //     }
  //     const permission = localStorage.getItem(localStorageKey.permission);
  //     if (permission && permission !== "/" && opts.path !== permission) {
  //       window.location.href = permission;
  //     }
  //   },
  // }
);
