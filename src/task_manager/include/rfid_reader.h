#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include "rfid_h/delegate.h"
#include "rfid_h/message.h"
#include "rfid_h/GClient.h"

class RFIDReader {
public:
    RFIDReader();
    ~RFIDReader();

    // 初始化：连接RS232设备，注册回调
    bool init();

    // 开始采集（非阻塞，立即返回）
    // 调用后后台线程持续盘存，标签数据通过回调实时写入缓存
    bool startScan();

    // 停止采集（非阻塞，立即返回）
    // 停止盘存，将当前缓存锁定为本次结果
    void stopScan();

    // 取本次采集结果（stopScan之后调用）
    std::vector<std::string> getResult();

    // 查询是否正在采集中
    bool isScanning() const { return scanning_.load(); }

    // 关闭连接，释放资源
    void close();

    bool isConnected() const { return client_ != nullptr && client_->isOpened; }

private:
    GClient*     client_;
    std::thread  inventory_thread_;
    std::atomic<bool> scanning_;    // true = 采集中
    std::atomic<bool> stop_request_;// 通知后台线程停止

    // 采集中回调实时写入
    std::vector<std::string> active_buffer_;
    std::mutex               active_mutex_;

    // stopScan后锁定，供getResult读取
    std::vector<std::string> result_buffer_;
    std::mutex               result_mutex_;

    static RFIDReader* instance_;

    void inventoryLoop();   // 后台线程函数

    static void onTagEpc(char* readerName, LogBaseEpcInfo msg);
    static void onTagEpcOver(char* readerName, LogBaseEpcOver msg);
    static void onDisconnected(char* readerName);
};