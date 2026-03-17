#ifndef _RCS_SERVER_H_
#define _RCS_SERVER_H_

#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <modbus.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "sys/syscall.h"
#include "common_msgs/rcs_cmd_data.h"
#include "common_msgs/ipc_res_data.h"
#include "common_msgs/common_haeder.h"
#include "std_msgs/Int8.h"

namespace rcs {
// modbus一帧报文的最大长度
#define MODBUS_TCP_MAX_ADU_LENGTH  260
#define DELETE_THREAD(ptr)    if((ptr) != nullptr){(ptr)->join(); delete (ptr); (ptr) = nullptr;}
// 将指定位置的二进制位设置为1
#define SET_BIT(x,y)    (x)|=(1<<(y))


//任务指令--枚举
enum RcsTaskCmdEnum
{
    TASKCMD_NONE,
    TASKCMD_PAUSE,  // 暂停任务
    TASKCMD_EXECUTE, // 执行任务
    TASKCMD_OBORT, // 中止任务
    TASKCMD_RECV, // 接收任务
    TASKCMD_REMOVE_STEP_PAUSE, // 
    TASKCMD_NUM
};
//链接状态--枚举
enum ConnectStateEnum
{
    CONNSTATE_DISCONNECT,
    CONNSTATE_WAIT_FOR_CONNECT,
    CONNSTATE_CONNECTED,
};

typedef struct {
    std::string rcs_ip;
    int rcs_port;
    int rcs_start_bits;
    int rcs_nb_bits;
    int rcs_start_input_bits;
    int rcs_nb_input_bits;
    int rcs_start_registers;
    int rcs_nb_registers;          // 管控输入的寄存器
    int rcs_start_input_registers;
    int rcs_nb_input_registers;    // 向管控输出的寄存器
    int rcs_recv_timeout = 0;      // unit: us
    int rcs_disconnect_count_max = 0;
} RCS_PARAMS;

class RcsInteraction
{
    public:
        RcsInteraction();
        ~RcsInteraction();

        void StartListening();
        void ServerThread();

    private:
        std::string RcsTaskCmdToString(const RcsTaskCmdEnum cmd);
        bool InitSocket();
        int UninitSocket();
        void ReadHoldingRegisters();
        void FillAndPubRcsTask(const uint16_t *const reg, const common_header::TaskCmdEnum& task_cmd);
        void UpdateInputRegisters();
        void FillIpcErrorCode(common_header::Ipc2RcsDataStru &data_ipc_to_rcs);
        void IpcResDataProcess(const common_msgs::ipc_res_data &data_res);
        bool IsPortAvailable(const std::string& ip, int port) ;

    private:
	    ros::NodeHandle nh_;
        ros::Publisher pub_rcs_task_;
        ros::Publisher pub_door_stop_;
        ros::Subscriber sub_ipc_res_data_;
        std::string topic_task_rcs_;
        std::chrono::time_point<std::chrono::system_clock> time_last_;
        std::chrono::time_point<std::chrono::system_clock> time_now_;
        std::chrono::duration<double> elapsed_seconds_;

        RCS_PARAMS params_rcs_;
        ConnectStateEnum status_connect_;
        bool flag_stop_thread_ = true;
        std::mutex lock_;
        std::thread *thread_rcs_server_ = nullptr;//子线程指针,初始默认为空指针

        modbus_t *m_ctx_ = nullptr;
        modbus_mapping_t *m_mb_mapping_ = nullptr;//Modbus内存映射指针
        int m_header_length_ = 0;
        int m_fd_ = -1;
        int count_one_step_task_ = 20; // unit: uint16_t
        RcsTaskCmdEnum cmd_rcs_current_ = TASKCMD_NONE;
        RcsTaskCmdEnum cmd_rcs_last_ = TASKCMD_NONE;
        common_header::Rcs2IpcTaskInfoStru rcs_task_info_;
        common_header::Ipc2RcsDataStru data_ipc_to_rcs_;
        
        enum CoilFlags {
        COIL_REAL_TIME_FLAG = 0,      // 实时通讯标志位 (地址0)
        COIL_EMERGENCY_STOP = 1,      // 急停标志位 (地址1)  
        COIL_TASK_READY = 2,          // 任务就绪标志位 (地址2)
        COIL_SYSTEM_READY = 3,        // 系统就绪标志位 (地址3)
        COIL_COUNT = 4                // 线圈总数
        };
        std::atomic<bool> real_time_flag_{false};
        std::atomic<bool> emergency_stop_{false};
        std::atomic<bool> task_ready_{false};
        std::atomic<bool> system_ready_{true};  // 默认系统就绪

        // 线圈处理方法
        void HandleCoilOperations();
        void PublishCoilStatus();
        void UpdateCoilStates();
};
}

#endif // _RCS_SERVER_H_