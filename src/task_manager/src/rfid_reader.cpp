#include "rfid_reader.h"
#include <ros/ros.h>
#include <string.h>
#include <unistd.h>

RFIDReader* RFIDReader::instance_ = nullptr;

RFIDReader::RFIDReader()
    : client_(nullptr), scanning_(false), stop_request_(false)
{
    instance_ = this;
}

RFIDReader::~RFIDReader() {
    close();
    instance_ = nullptr;
}

// ── 初始化 ────────────────────────────────────────────────────────────────────

bool RFIDReader::init() {
    const char* port = "/dev/ttyUSB0:115200";
    ROS_INFO("[RFID] Connecting via RS232 %s ...", port);

    client_ = OpenRS232(port, 5);
    if (client_ == nullptr) {
        ROS_ERROR("[RFID] Failed to connect.");
        return false;
    }
    ROS_INFO("[RFID] Connected.");

    RegCallBack(client_, ETagEpcLog,       (void*)onTagEpc);
    RegCallBack(client_, ETagEpcOver,      (void*)onTagEpcOver);
    RegCallBack(client_, ETcpDisconnected, (void*)onDisconnected);  // 串口断开事件复用此枚举

    // 清除读写器残留盘存状态
    MsgBaseStop stop;
    memset(&stop, 0, sizeof(stop));
    SendSynMsg(client_, EMESS_BaseStop, &stop);

    return true;
}

// ── startScan ─────────────────────────────────────────────────────────────────

bool RFIDReader::startScan() {
    if (!isConnected()) {
        ROS_ERROR("[RFID] startScan() called but not connected.");
        return false;
    }
    if (scanning_.load()) {
        ROS_WARN("[RFID] startScan() called but already scanning, ignored.");
        return false;
    }

    // 确保上一次线程已完全回收，防止重复赋值 std::thread 导致 terminate()
    if (inventory_thread_.joinable()) {
        inventory_thread_.join();
    }

    // 清空缓存，准备新一轮采集
    {
        std::lock_guard<std::mutex> lock(active_mutex_);
        active_buffer_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_buffer_.clear();
    }

    stop_request_ = false;
    scanning_     = true;

    inventory_thread_ = std::thread(&RFIDReader::inventoryLoop, this);
    ROS_INFO("[RFID] Scan started.");
    return true;
}

// ── inventoryLoop（后台线程）─────────────────────────────────────────────────

void RFIDReader::inventoryLoop() {
    // 发送盘存命令
    MsgBaseInventoryEpc msg;
    memset(&msg, 0, sizeof(msg));
    msg.AntennaEnable = AntennaNo_2;
    msg.InventoryMode = InventoryMode_Inventory;  // 持续盘存模式
    msg.ReadTid.Mode  = 0;
    msg.ReadTid.Len   = 6;

    SendSynMsg(client_, EMESS_BaseInventoryEpc, &msg);
    if (msg.rst.RtCode != 0) {
        ROS_ERROR("[RFID] Failed to start inventory: %s", msg.rst.RtMsg);
        scanning_ = false;
        return;
    }

    // 等待外部调用 stopScan() 或断线触发 stop_request_
    while (!stop_request_.load()) {
        usleep(50000);  // 50ms 轮询一次，CPU 占用极低
    }

    // 发送停止命令（断线时 client_ 可能已为 nullptr，需判断）
    if (client_ != nullptr && client_->isOpened) {
        MsgBaseStop stop;
        memset(&stop, 0, sizeof(stop));
        SendSynMsg(client_, EMESS_BaseStop, &stop);
        if (stop.rst.RtCode != 0) {
            ROS_WARN("[RFID] Stop inventory failed: %s", stop.rst.RtMsg);
        }
    } else {
        ROS_WARN("[RFID] Skip stop command, connection already lost.");
    }

    // 将 active_buffer_ 锁定到 result_buffer_
    {
        std::lock_guard<std::mutex> la(active_mutex_);
        std::lock_guard<std::mutex> lr(result_mutex_);
        result_buffer_ = active_buffer_;
    }

    ROS_INFO("[RFID] Scan stopped, found %zu tag(s).", result_buffer_.size());
    scanning_ = false;  // 最后由线程自己置 false
}

// ── stopScan ──────────────────────────────────────────────────────────────────

void RFIDReader::stopScan() {
    if (!scanning_.load()) {
        ROS_WARN("[RFID] stopScan() called but not scanning.");
        return;
    }

    stop_request_ = true;

    // 等待后台线程完成停止流程（通常 < 100ms）
    if (inventory_thread_.joinable()) {
        inventory_thread_.join();
    }
}

// ── getResult ─────────────────────────────────────────────────────────────────

std::vector<std::string> RFIDReader::getResult() {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return result_buffer_;
}

// ── close ─────────────────────────────────────────────────────────────────────

void RFIDReader::close() {
    if (scanning_.load()) {
        stopScan();
    }
    // 确保线程完全退出
    if (inventory_thread_.joinable()) {
        inventory_thread_.join();
    }
    if (client_ != nullptr && client_->isOpened) {
        ROS_INFO("[RFID] Closing connection.");
        Close(client_);
    }
    client_ = nullptr;
}

// ── 静态回调 ──────────────────────────────────────────────────────────────────

void RFIDReader::onTagEpc(char* readerName, LogBaseEpcInfo msg) {
    if (instance_ == nullptr || msg.Result != 0) return;
    if (!instance_->scanning_.load()) return;

    std::string epc(reinterpret_cast<const char*>(msg.Epc));
    std::lock_guard<std::mutex> lock(instance_->active_mutex_);

    for (const auto& existing : instance_->active_buffer_) {
        if (existing == epc) return;
    }
    instance_->active_buffer_.push_back(epc);
    ROS_INFO("[RFID] Tag detected - EPC: %s", epc.c_str());
}

void RFIDReader::onTagEpcOver(char* readerName, LogBaseEpcOver msg) {
    ROS_DEBUG("[RFID] TagEpcOver.");
}

void RFIDReader::onDisconnected(char* readerName) {
    ROS_WARN("[RFID] Disconnected: %s", readerName);
    if (instance_ != nullptr) {
        instance_->client_       = nullptr;
        instance_->stop_request_ = true;  // 通知 inventoryLoop 退出循环
        // scanning_ 由 inventoryLoop 线程自己在退出时置 false，不在此处修改
    }
}