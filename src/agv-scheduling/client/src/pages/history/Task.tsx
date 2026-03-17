import dayjs from "dayjs";
import { useCallback, useEffect, useState } from "react";
import { App, Button, Space, Table, Tag } from "antd";
import { Copy, ClipboardList } from "lucide-react";
import { taskApi } from "@/network/api";
import { copyTextToClipboard } from "@/lib";
import type { TaskRecord } from "@/types/task";
import type { Pagination } from "@/types/pagination";
import type { FilterValue, SorterResult } from "antd/es/table/interface";
import type { ColumnsType, TablePaginationConfig } from "antd/es/table";

export default function Meters() {
  const { notification, message } = App.useApp();

  // 分页设置
  const [pagination, setPagination] = useState<Pagination<TaskRecord>>({
    page: 1,
    pageSize: 10,
    total: 0,
    data: [],
  });

  // 行数据
  const [rows, setRows] = useState<TaskRecord[]>([]);

  // 表格列
  const columns: ColumnsType<TaskRecord> = [
    {
      title: "序号",
      align: "center",
      width: 60,
      render(_, __, index) {
        return (pagination.page - 1) * pagination.pageSize + (index + 1) + ".";
      },
    },
    {
      title: "AGV车辆",
      align: "center",
      dataIndex: "agv_name",
    },
    {
      title: "任务类型",
      align: "center",
      dataIndex: "type",
      filters: [
        { text: "取货卸货", value: "pickDrop" },
        { text: "移动", value: "move" },
        { text: "举升", value: "raise" },
        { text: "充电", value: "charge" },
        { text: "取消充电", value: "stop_charge" },
        { text: "取消任务", value: "stop" },
        { text: "卷帘门:开", value: "open_door" },
        { text: "卷帘门:关", value: "close_door" },
      ],
      render(value) {
        const { color, text } = getTypeStyle(value);
        return <Tag color={color}>{text}</Tag>;
      },
    },
    {
      title: "耗时(ms)",
      align: "center",
      dataIndex: "duration",
      sorter: true,
    },
    {
      title: "创建日期",
      align: "center",
      dataIndex: "created_at",
      sorter: true,
      render(value) {
        return dayjs(value).format("YYYY-MM-DD HH:mm:ss");
      },
    },
    {
      title: "操作",
      align: "center",
      render(record: TaskRecord) {
        return (
          <Space>
            <Button
              type="text"
              title="任务代码"
              icon={<ClipboardList size={16} />}
              disabled={!record.code || record.code.length === 0}
              onClick={() => onCopyToClipboard(1, record.code ?? "")}
            />
            <Button
              type="link"
              title="任务解析"
              icon={<Copy size={16} />}
              disabled={!record.body || record.body.length === 0}
              onClick={() => onCopyToClipboard(2, record.body ?? "")}
            />
          </Space>
        );
      },
    },
  ];

  // 获取任务类型样式
  const getTypeStyle = (type: string) => {
    // 取消任务
    switch (type) {
      case "stop":
        return { color: "red", text: "取消任务" };

      case "pickDrop":
        return { color: "cyan", text: "取货卸货" };

      case "raise":
        return { color: "gold", text: "举升" };

      case "charge":
        return { color: "green", text: "充电" };

      case "stop_charge":
        return { color: "purple", text: "取消充电" };

      case "open_door":
        return { color: "magenta", text: "卷帘门:开" };

      case "close_door":
        return { color: "magenta", text: "卷帘门:关" };

      default:
        return { color: "geekblue", text: "移动" };
    }
  };

  // 复制到剪切板
  const onCopyToClipboard = useCallback(
    async (type: number, text: string) => {
      if (await copyTextToClipboard(text)) {
        notification.success({
          message: `已复制${type === 1 ? '"任务代码"' : '"任务解析"'}`,
        });
      }
    },
    [notification]
  );

  // 请求获取数据
  const refreshData = useCallback(
    (args: {
      page: number;
      pageSize: number;
      typeFilters?: string[];
      sorterField?: string;
      order?: string;
    }) => {
      const {
        page,
        pageSize,
        typeFilters,
        sorterField = "created_at",
        order = "desc",
      } = args;
      taskApi
        .getTaskLogs({
          page,
          pageSize,
          typeFilters: typeFilters?.join(",") ?? "",
          sorterField,
          order,
        })
        .then((res) => {
          if (res.status === 200) {
            setPagination(res.data);
            setRows(res.data.data);
          } else {
            message.error(res.statusText);
          }
        });
    },
    [message]
  );

  // 表格改变事件
  const onTableChange = useCallback(
    (
      tpc: TablePaginationConfig,
      filter: Record<string, FilterValue | null>,
      sorter: SorterResult<TaskRecord>
    ) => {
      refreshData({
        page: tpc.current ?? 1,
        pageSize: tpc.pageSize ?? 10,
        typeFilters: filter.type as string[],
        sorterField: sorter.field as string,
        order: sorter.order as string,
      });
    },
    [refreshData]
  );

  // 首次加载，获取设备列表
  useEffect(() => {
    refreshData({ page: 1, pageSize: 10 });
  }, [refreshData]);

  return (
    <div className="px-6 py-4 max-h-full overflow-y-auto">
      <div className="bg-white rounded-xl shadow-md p-6 pb-0">
        <Table
          columns={columns}
          dataSource={rows}
          pagination={{
            total: pagination.total,
            showSizeChanger: true,
            pageSizeOptions: [5, 10, 20],
            showTotal: (total) => (
              <span className="text-gray-500">共 {total} 条</span>
            ),
          }}
          rowKey={(record) => record.id}
          rowClassName={(_, i) => (i % 2 == 1 ? "bg-[#fafafa]" : "")}
          showSorterTooltip={{ target: "sorter-icon" }}
          scroll={{ x: "max-content" }}
          onChange={(tpc, ft, st) => {
            onTableChange(tpc, ft, st as SorterResult<TaskRecord>);
          }}
        />
      </div>
    </div>
  );
}
