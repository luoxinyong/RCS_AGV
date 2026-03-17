import { Outlet } from "react-router-dom";
import Header from "../components/Header";
import Sidebar from "../components/Sidebar";

export default function Layout() {
  return (
    <div className="h-screen flex flex-col p-3 bg-[#eff6ff] min-w-[768px]">
      <Header />
      <div className="flex-1 overflow-hidden mt-2 flex">
        <div className="flex-1">
          <Sidebar />
        </div>
        <main className="w-[calc(100%_-_14rem)] ml-2">
          <Outlet />
        </main>
      </div>
    </div>
  );
}
