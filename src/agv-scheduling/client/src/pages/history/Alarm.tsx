import dayjs from "dayjs";
import { useCallback, useEffect, useState } from "react";
import { App, Button, Space, Table, Tag } from "antd";
import { Copy, ClipboardList } from "lucide-react";
import { alarmApi } from "@/network/api";
import { copyTextToClipboard } from "@/lib";
import type { AlarmRecord } from "@/types/alarm";
import type { Pagination } from "@/types/pagination";
import type { SorterResult } from "antd/es/table/interface";
import type { ColumnsType, TablePaginationConfig } from "antd/es/table";

export default function Alarm() {
  const { notification, message } = App.useApp();

  // 分页设置
  const [pagination, setPagination] = useState<Pagination<AlarmRecord>>({
    page: 1,
    pageSize: 10,
    total: 0,
    data: [],
  });

  // 行数据
  const [rows, setRows] = useState<AlarmRecord[]>([]);

  // 表格列
  const columns: ColumnsType<AlarmRecord> = [
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
      title: "检查状态",
      align: "center",
      dataIndex: "tr_check_state",
      render: (v) => <Tag color={v === 3 ? "red" : "green"}>{v}</Tag>,
    },
    {
      title: "检查错误",
      align: "center",
      dataIndex: "tr_check_error",
      render: (v) => (
        <Tag color={v === 0 ? "green" : "red"}>
          {v === 1000 ? "发送失败" : v}
        </Tag>
      ),
    },
    {
      title: "错误码",
      align: "center",
      dataIndex: "err1",
      render: (v) => <Tag color={v === 0 ? "green" : "red"}>{v}</Tag>,
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
      render(record: AlarmRecord) {
        return (
          <Space>
            <Button
              type="text"
              title="状态代码"
              icon={<ClipboardList size={16} />}
              disabled={!record.code || record.code.length === 0}
              onClick={() => onCopyToClipboard(1, record.code ?? "")}
            />
            <Button
              type="link"
              title="状态解析"
              icon={<Copy size={16} />}
              disabled={!record.body || record.body.length === 0}
              onClick={() => onCopyToClipboard(2, record.body ?? "")}
            />
          </Space>
        );
      },
    },
  ];

  // 复制到剪切板
  const onCopyToClipboard = useCallback(
    async (type: number, text: string) => {
      if (await copyTextToClipboard(text)) {
        notification.success({
          message: `已复制${type === 1 ? '"状态代码"' : '"状态解析"'}`,
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
      sorterField?: string;
      order?: string;
    }) => {
      const {
        page,
        pageSize,
        sorterField = "created_at",
        order = "desc",
      } = args;
      alarmApi.getAlarms({ page, pageSize, sorterField, order }).then((res) => {
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
    (tpc: TablePaginationConfig, sorter: SorterResult<AlarmRecord>) => {
      refreshData({
        page: tpc.current ?? 1,
        pageSize: tpc.pageSize ?? 10,
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
          onChange={(tpc, _, st) =>
            onTableChange(tpc, st as SorterResult<AlarmRecord>)
          }
        />
      </div>
    </div>
  );
}
