#include "plc_interaction.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include "cmath"

namespace mio
{
    
// 整个构造函数会创建四个线程，分别负责向PLC导航、执行机构的命令发送以及接收来自PLC的指令
// 执行单元	                    功能描述                                线程归属	      激活条件	              执行特性
// RecvNavThread	    接收来自PLC的导航信息并解析                   专用线程	        创建后立即执行	         持续循环运行
// RecvActionThread	    接收来自PLC的执行信息并解析                   专用线程	        创建后立即执行	         持续循环运行
// NavDataProcess	    接收来自导航系统（调度）导航指令并发送给PLC	     主线程	         消息到达/nav_data	      事件驱动执行
// ActionDataProcess	接收来自执行系统（调度）执行指令并发送给PLC	     主线程	         消息到达/action_data	  事件驱动执行    
PlcInteraction::PlcInteraction()
{
    // 参数配置,从launch指定的yaml文件读取
    ros::NodeHandle nh;
    nh.param<std::string>("/ip_address_plc", ip_address_plc_, std::string("192.168.1.200"));
    // 导航的行为(前进或后退)对应的端口号
    nh.param<int>("/socket_server_port_nav", port_socket_server_nav_, 1220);
    nh.param<int>("/socket_client_port_nav", port_socket_client_nav_, 1221);
    //动作行为（举升）对应的端口号
    nh.param<int>("/socket_server_port_action", port_socket_server_action_, 1110);
    nh.param<int>("/socket_client_port_action", port_socket_client_action_, 1111);
    // 编码器数据传输标志位？
    nh.param<bool>("/flag_plc_send_encoder_fix", flag_plc_send_encoder_fix_, false);
    // 编码器数据对应的话题名
    nh.param<std::string>("/topic_encoder", topic_encoder_, "encoder");
    // 速度数据对应的话题名
    nh.param<std::string>("/topic_vel", topic_vel_, "vel");
    // imu数据对应的话题名
    nh.param<std::string>("/topic_imu", topic_imu_, "imu_plc");
    // 里程计数据对应的话题名
    nh.param<std::string>("/topic_odom", topic_odom_, "odom");
    ROS_INFO(" ip: %s, topic: [%s, %s, %s, %s], nav_port: [%d, %d], action_port: [%d, %d], ", ip_address_plc_.c_str(),
             topic_encoder_.c_str(), topic_vel_.c_str(), topic_imu_.c_str(), topic_odom_.c_str(),
             port_socket_server_nav_, port_socket_client_nav_, port_socket_server_action_, port_socket_client_action_);

    // UDP网络通信连接,负责接收来自PLC的数据并进行解析
    udp_client_nav_ptr_ =
        std::shared_ptr<UdpClient>(new UdpClient(ip_address_plc_, port_socket_server_nav_, port_socket_client_nav_));
    udp_client_nav_ptr_->StartConnection(true);
    // 导航UDP子线程实例化,负责接收来自PLC的导航信息并解析
    thread_recv_nav_ = new std::thread(&PlcInteraction::RecvNavThread, this);

    // UDP网络通信连接,负责接收来自PLC的执行信息并解析
    udp_client_action_ptr_ = std::shared_ptr<UdpClient>(
        new UdpClient(ip_address_plc_, port_socket_server_action_, port_socket_client_action_));
    udp_client_action_ptr_->StartConnection(true);
    // 动作UDP子线程实例化,负责接收来自PLC的执行信息并解析
    thread_recv_action_ = new std::thread(&PlcInteraction::RecvActionThread, this);

    pub_encoder_ = nh.advertise<geometry_msgs::PoseStamped>(topic_encoder_, 10);
    pub_vel_ = nh.advertise<geometry_msgs::Twist>(topic_vel_, 10);
    pub_imu_ = nh.advertise<sensor_msgs::Imu>(topic_imu_, 10);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>(topic_odom_, 10);

    pub_res_action_data_ = nh.advertise<common_msgs::plc_res_action_data>("plc_res_action_data", 10);
    pub_res_data_ = nh.advertise<common_msgs::plc_res_nav_data>("plc_res_nav_data", 10);
    // 接收来自task-manager的导航指令并下发给plc
    sub_nav_ = nh.subscribe("/nav_data", 1000, &PlcInteraction::NavDataProcess, this);
    // 接收来自task-manager的执行指令并下发给plc
    sub_action_ = nh.subscribe("/action_data", 1000, &PlcInteraction::ActionDataProcess, this);
}

void PlcInteraction::NavDataProcess(const common_msgs::nav_data& data_nav)
{
    unsigned char msg_data[512] = {0};
    int msg_len = sizeof(IPC_NAV_MSG);

    nav_data_ipc_.id_current_node = static_cast<int32_t>(data_nav.id_task);
    nav_data_ipc_.type_current_node = static_cast<int32_t>(data_nav.type_current_task);
    nav_data_ipc_.speed_current_node = static_cast<int32_t>(1000.0 * data_nav.max_speed_current_task);

    nav_data_ipc_.type_next_node = static_cast<int32_t>(data_nav.type_next_task);
    nav_data_ipc_.speed_next_node = static_cast<int32_t>(1000.0 * data_nav.max_speed_next_task);
    nav_data_ipc_.data_next_node = static_cast<int32_t>(1000.0 * data_nav.radius_sign_next_task);

    nav_data_ipc_.v_x = static_cast<int32_t>(1000.0 * data_nav.cmd_vel.linear.x);
    nav_data_ipc_.v_y = static_cast<int32_t>(1000.0 * data_nav.cmd_vel.linear.y);
    nav_data_ipc_.w = static_cast<int32_t>(1000.0 * data_nav.cmd_vel.angular.z * 180.0 / M_PI);

    nav_data_ipc_.loc_x = static_cast<int32_t>(1000.0 * data_nav.point_current.position.x);
    nav_data_ipc_.loc_y = static_cast<int32_t>(1000.0 * data_nav.point_current.position.y);
    nav_data_ipc_.loc_theta = static_cast<int32_t>(1000.0 * data_nav.yaw_current_pose * 180.0 / M_PI);

    nav_data_ipc_.delta = static_cast<int32_t>(1000.0 * data_nav.delta_center);  // 车体中心到路线的投影距离
    nav_data_ipc_.delta_theta = static_cast<int32_t>(1000.0 * data_nav.delta_angle);  // 车头方向与路径朝向的夹角
    nav_data_ipc_.p_delta = static_cast<int32_t>(1000.0 * data_nav.delta_head);       // 运动方向头部偏差
    nav_data_ipc_.L_delta = static_cast<int32_t>(1000.0 * data_nav.delta_tail);       // 运动方尾部偏差

    nav_data_ipc_.remain_turn_theta = static_cast<int32_t>(1000.0 * data_nav.angle_diff_spin_around * 180.0 / M_PI);
    nav_data_ipc_.radius_turn = static_cast<int32_t>(1000.0 * data_nav.radius_turn);
    nav_data_ipc_.length_turn = static_cast<int32_t>(1000.0 * data_nav.length_turn);
    nav_data_ipc_.dis_to_stop_node = static_cast<int32_t>(1000.0 * data_nav.dis_to_stop_node);
    nav_data_ipc_.dis_to_type_switch_node = static_cast<int32_t>(1000.0 * data_nav.dis_to_type_switch_node);
    nav_data_ipc_.flag_need_avoid = data_nav.flag_need_avoid;

    nav_data_ipc_.ctrl_charge= data_nav.flag_charge;
    nav_data_ipc_.obs_area = data_nav.obs_area;
    nav_data_ipc_.reserve_1 = data_nav.flag_door_stop;
    nav_data_ipc_.flag_enter_elevator = data_nav.flag_enter_elevator;

    nav_data_ipc_.list_error = data_nav.error_code;
    nav_data_ipc_.dis_remain_curr_node = static_cast<int32_t>(1000.0 * data_nav.dis_remain_curr_node);

    nav_data_ipc_.flag_detect_pallet = data_nav.flag_detect_pallet;
    nav_data_ipc_.flag_pick_task_valid = data_nav.flag_pick_task_valid;
    nav_data_ipc_.x_detect = data_nav.x_detect;
    nav_data_ipc_.y_detect = data_nav.y_detect;
    nav_data_ipc_.theta_detect = data_nav.theta_detect;

    nav_data_ipc_.x_preview_point = static_cast<int32_t>(1000.0 * data_nav.point_lookahead.x);
    nav_data_ipc_.y_preview_point = static_cast<int32_t>(1000.0 * data_nav.point_lookahead.y);

    nav_data_ipc_.x_base_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_base.x);
    nav_data_ipc_.y_base_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_base.y);
    nav_data_ipc_.x_head_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_head.x);
    nav_data_ipc_.y_head_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_head.y);
    nav_data_ipc_.x_tail_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_tail.x);
    nav_data_ipc_.y_tail_project_point = static_cast<int32_t>(1000.0 * data_nav.project_point_tail.y);

    memcpy(msg_data, (char*)(&nav_data_ipc_), msg_len);

    if (!udp_client_nav_ptr_->ConstructAndSendMsg(msg_len, msg_data)) {
        ROS_ERROR("R_err_8: send nav data to plc failed ...... ");
    }
    // else{
    //     ROS_INFO("send nav data to plc success ...... ");
    //     std::cout<<"nav_data_ipc_.id_current_node: "<<nav_data_ipc_.id_current_node<<
    //         " nav_data_ipc_.type_current_node: "<<nav_data_ipc_.type_current_node<<
    //         " nav_data_ipc_.speed_current_node: "<<nav_data_ipc_.speed_current_node<<std::endl;
    // }
}

void PlcInteraction::ActionDataProcess(const common_msgs::action_data& data_action)
{
    unsigned char msg_data[512] = {0};
    int msg_len = sizeof(IPC_ACTION_MSG);

    action_data_ipc_.type_execute_action = data_action.type_execute_action;

    for (size_t i = 0; i < data_action.action_tasks.size(); ++i) {
        action_data_ipc_.attributes[i].action_cmd = data_action.action_tasks[i].action_cmd;
        action_data_ipc_.attributes[i].action_value = data_action.action_tasks[i].action_value;
        action_data_ipc_.attributes[i].action_id = data_action.action_tasks[i].action_id;
    }

    memcpy(msg_data, (char*)(&action_data_ipc_), msg_len);

    if (!udp_client_action_ptr_->ConstructAndSendMsg(msg_len, msg_data)) {
        ROS_ERROR("R_err_10: send action data to plc failed ...... ");
    }
    // else{
    //     ROS_INFO("send action data to plc success ...... ");
    //     std::cout<<"action_data_ipc_.type_execute_action: "<<action_data_ipc_.type_execute_action<<
    //         " action_data_ipc_.attributes[0].action_cmd: "<<action_data_ipc_.attributes[0].action_cmd<<
    //         " action_data_ipc_.attributes[0].action_value: "<<action_data_ipc_.attributes[0].action_value<<
    //         " action_data_ipc_.attributes[0].action_id: "<<action_data_ipc_.attributes[0].action_id<<std::endl;
    // }
}
                                                                                                                                                                                                                                                                                                                
//查找消息头函数体
bool PlcInteraction::SearchHeader(std::vector<unsigned char>& data)
{
    auto it = data.begin();
    while (it != data.end()) {
        // ROS_INFO(" check byte: %02X ", *it);
        if (((it + 1) != data.end()) && (__MIO_MSG_HEADER1__ == *it) && (__MIO_MSG_HEADER2__ == *(it + 1))) {
            data.erase(data.begin(), it);
            // ROS_INFO(" header find find ...... ");
            return true;
        }
        ++it;
    }
    data.clear();
    ROS_ERROR("R_err_9: not find header ......");
    return false;
}
//解析消息函数体
bool PlcInteraction::ParsePacket(const std::vector<unsigned char>& queue, HeaderData& header_data,
                                 unsigned char* buffer, size_t& packet_length)
{
    //解析消息，先判断消息长度是否足够
    if (queue.size() < sizeof(HeaderData)) {
        return false;
    }
    //解析消息头，把消息头的数据保存到header_data里面
    memcpy(&header_data, queue.data(), sizeof(HeaderData));
    // 解析消息体，先判断消息体长度是否足够，通讯协议里面已经定义了消息体的长度=消息头长度+帧体数据长度+1（校验位）
    packet_length = header_data.msg_len + sizeof(HeaderData) + 1;
    if (queue.size() < packet_length) {
        return false;
    }
    // 校验，如果校验失败，则删除校验以前的所有数据
    //      if(!Checkout(queue, packet_length))
    //      {
    //         queue.erase(queue.begin(), queue.begin() + packet_length);
    //         ROS_INFO("check error");
    //         return false;
    //      }
    // 解析消息体，把消息体的帧体数据（有效数据段）保存到buffer里面
    std::copy(queue.begin() + sizeof(HeaderData), queue.begin() + packet_length - 1, buffer);
    return true;
}
//显示接收消息的日志函数体，传入的参数是一个名字
// 这里传入的__FUNCTION__，表示当前函数的名称,也就是RecvNavThread或者RecvActionThread
void PlcInteraction::ShowRecvLog(const std::string& name_call)
{
    static int count_print = 0;
    count_print++;
    if (count_print > 300) {
        count_print = 0;
        ROS_INFO("R_step_3: %s ...... ", name_call.c_str());
    }
}
//导航子线程的接收处理函数，
/*
*创建一个持续运行的线程，用于接收和处理来自PLC的UDP导航消息 
*使用 m_stop_recv_nav_ 标志控制线程运行/停止，线程退出时会打印关闭信息
 */
void PlcInteraction::RecvNavThread()
{
    // 1、线程初始化
    ROS_INFO("R_step_0: nav recv thread has been started!");
    std::vector<unsigned char> recv_msg_queue;//接收消息队列
    recv_msg_queue.clear();//接收消息队列清空
    unsigned char buffer_nav[10000] = {0};//导航消息接收缓存
    int len = 0, ret = 0;

    m_stop_recv_nav_ = false;//导航接收线程运行标志，初始为false说明正在运行中
    // 2、线程主循环
    while (!m_stop_recv_nav_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));//10ms运行一次
        nav_connect_status_ = udp_client_nav_ptr_->ConnectStatus();//检查连接状态
        if (nullptr == udp_client_nav_ptr_ || CONNSTATE_CONNECTED != nav_connect_status_) {
            ROS_ERROR("R_err_0:  udp not connect ");
            recv_msg_queue.clear();
        }
        len = udp_client_nav_ptr_->GetSocketData(recv_msg_queue);//获取数据
        if (len <= 0) {
            ROS_ERROR("R_err_1: GetSocketData len less than or equal 0 ");
            continue;
        }
        // ROS_INFO("R_step_1: recv_nav_msg_queue.size() = %d", recv_msg_queue.size());

        // for (size_t i = 0; i < recv_msg_queue.size(); ++i) {
        //     ROS_INFO(" [%d]: %x", i+1, recv_msg_queue[i]);
        // }

        std::vector<unsigned char>::iterator iter;
        while (true) {
            if (!recv_msg_queue.size()) {//保证消息队列不为空
                break;
            }
            if (!SearchHeader(recv_msg_queue)) {//查找消息头，本质上就是用迭代器一个个查看固定帧头FFFA
                ROS_ERROR("R_err_2:  nav no find header ...... ");
                break;
            }
            size_t packet_length = 0;
            if (!ParsePacket(recv_msg_queue, header_data_, buffer_nav, packet_length)) {//解析消息
                ROS_ERROR("R_err_3: ParsePacket error ...... ");
                break;
            }
            recv_msg_queue.erase(recv_msg_queue.begin(), recv_msg_queue.begin() + packet_length);//删除已解析的消息     
            ShowRecvLog(__FUNCTION__);
            //有效数据长度=总包长度-消息头长度-1
            size_t data_length = packet_length - sizeof(HeaderData) - 1;
            // 再次进行判断
            if (!HandleRecvNavMsg(buffer_nav, data_length)) {
                ROS_ERROR("R_err_7: Handle nav recv msg failed ...... ");
            }
        }
    }
    ROS_INFO("Rec_step_10: nav recv thread has been shutdown!");
}
// 执行动作的子线程绑定的处理函数
void PlcInteraction::RecvActionThread()
{
    ROS_INFO("R_step_20: action recv thread has been started!");
    std::vector<unsigned char> recv_action_msg_queue;
    recv_action_msg_queue.clear();
    unsigned char buffer_action[10000] = {0};
    int len = 0, ret = 0;

    m_stop_recv_action_ = false;
    while (!m_stop_recv_action_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        action_connect_status_ = udp_client_action_ptr_->ConnectStatus();
        if (nullptr == udp_client_action_ptr_ || CONNSTATE_CONNECTED != action_connect_status_) {
            recv_action_msg_queue.clear();
        }
        len = udp_client_action_ptr_->GetSocketData(recv_action_msg_queue);
        if (len <= 0) {
            continue;
        }
        // ROS_INFO("R_step_21: recv_action_msg_queue.size() = %d", recv_action_msg_queue.size());

        std::vector<unsigned char>::iterator iter;
        while (true) {
            if (!recv_action_msg_queue.size()) {
                break;
            }
            if (!SearchHeader(recv_action_msg_queue)) {
                ROS_ERROR("R_err_22:  action no find header ...... ");
                break;
            }
            size_t packet_length = 0;
            if (!ParsePacket(recv_action_msg_queue, header_data_, buffer_action, packet_length)) {
                ROS_ERROR("R_err_23: ParsePacket error ...... ");
                break;
            }
            recv_action_msg_queue.erase(recv_action_msg_queue.begin(), recv_action_msg_queue.begin() + packet_length);
            ShowRecvLog(__FUNCTION__);
            //注册和初始化成功，开始正常数据接收处理
            if (!HandleRecvActionMsg(buffer_action, packet_length)) {
                ROS_ERROR("R_err_27: Handle action recv msg failed ...... ");
            }
        }
    }
    ROS_INFO("Rec_step_20: action recv thread has been shutdown!");
}

void PlcInteraction::PublishImuData(const PLC_NAV_MSG& nav_data_plc)
{
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "imu_link";
    imu_data.orientation.x = 0.0001 * nav_data_plc.Quaterniond_x;
    imu_data.orientation.y = 0.0001 * nav_data_plc.Quaterniond_y;
    imu_data.orientation.z = 0.0001 * nav_data_plc.Quaterniond_z;
    imu_data.orientation.w = 0.0001 * nav_data_plc.Quaterniond_w;
    imu_data.angular_velocity.x = 0.001 * nav_data_plc.angular_velocity_x;
    imu_data.angular_velocity.y = 0.001 * nav_data_plc.angular_velocity_y;
    imu_data.angular_velocity.z = 0.001 * nav_data_plc.angular_velocity_z / 180.0 * M_PI;
    pub_imu_.publish(imu_data);
}

void PlcInteraction::PublishEncoderData(const PLC_NAV_MSG& nav_data_plc)
{
    geometry_msgs::PoseStamped encoder_data;
    encoder_data.header.stamp = ros::Time::now();
    encoder_data.header.frame_id = "base_link";
    encoder_data.pose.position.x = nav_data_plc.encoder_left;          // [车子正前方左侧轮子]
    encoder_data.pose.position.y = nav_data_plc.encoder_right;         // [车子正前方右侧轮子]
    encoder_data.pose.position.z = 0.01 * nav_data_plc.linear_center;  // 车体中心线速度

    encoder_data.pose.orientation.x =
        0.01 * nav_data_plc.speed_left;  // 代表差分轮1的速度 前进两个轮子速度都是正，后退都是负
    encoder_data.pose.orientation.y = 0.01 * nav_data_plc.speed_right;        // 代表差分轮2的速度
    encoder_data.pose.orientation.z = 0.01 * nav_data_plc.angle_left_front;   // 代表方向轮1的角度
    encoder_data.pose.orientation.w = 0.01 * nav_data_plc.angle_right_front;  // 代表方向轮2的角度
    pub_encoder_.publish(encoder_data);
}

void PlcInteraction::PublishOdomDataAndTf(const PLC_NAV_MSG& nav_data_plc)
{
    nav_msgs::Odometry odom_msgs;
    odom_msgs.header.stamp = ros::Time::now();
    odom_msgs.header.frame_id = "odom";
    odom_msgs.child_frame_id = "base_footprint";
    odom_msgs.twist.twist.linear.x = 0.001 * nav_data_plc.v_x;
    odom_msgs.twist.twist.linear.y = 0.001 * nav_data_plc.v_y;
    odom_msgs.twist.twist.angular.z = 0.001 * nav_data_plc.w / 180.0 * M_PI;

    if (std::fabs(odom_msgs.twist.twist.linear.x) < 0.008) {
        odom_msgs.twist.twist.linear.x = 0;
    }
    if (std::fabs(odom_msgs.twist.twist.linear.y) < 0.008) {
        odom_msgs.twist.twist.linear.y = 0;
    }

    //航迹推算
    ros::Time current_time = odom_msgs.header.stamp;
    static ros::Time last_time = current_time;
    static double x = 0.0;
    static double y = 0.0;
    static double theta = 0.0;

    double dt = (current_time - last_time).toSec();
    double delta_x = odom_msgs.twist.twist.linear.x * dt;
    double delta_y = odom_msgs.twist.twist.linear.y * dt;
    double delta_theta = odom_msgs.twist.twist.angular.z * dt;

    theta += delta_theta;
    //角度归一化[-pi,pi)
    if (M_PI <= theta) {
        theta = theta - 2 * M_PI;
    } else if (-M_PI > theta) {
        theta = theta + 2 * M_PI;
    }

    x += delta_x * cos(theta) - delta_y * sin(theta);
    y += delta_x * sin(theta) + delta_y * cos(theta);
    odom_msgs.pose.pose.position.x = x;
    odom_msgs.pose.pose.position.y = y;
    odom_msgs.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    odom_msgs.pose.pose.orientation = odom_quat;
    pub_odom_.publish(odom_msgs);

    //发布坐标变换
    // static tf::TransformBroadcaster tf_broadcaster;
    // tf::StampedTransform transform;
    // transform.setOrigin(
    //     tf::Vector3(odom_msgs.pose.pose.position.x, odom_msgs.pose.pose.position.y, odom_msgs.pose.pose.position.z));
    // // 设置旋转部分（四元数）
    // tf::Quaternion rotation(odom_msgs.pose.pose.orientation.x, odom_msgs.pose.pose.orientation.y,
    //                         odom_msgs.pose.pose.orientation.z, odom_msgs.pose.pose.orientation.w);
    // transform.setRotation(rotation);
    // transform.stamp_ = ros::Time::now();
    // transform.frame_id_ = odom_msgs.header.frame_id;
    // transform.child_frame_id_ = odom_msgs.child_frame_id;
    // tf_broadcaster.sendTransform(transform);
    last_time = current_time;
}
// 从底层PLC获取传感器数据及车辆状态信息，经过处理后发布给上层任务管理系统task-manager
bool PlcInteraction::HandleRecvNavMsg(const unsigned char* buffer, const int len)
{   
    // 检查功能码是否正确
    if (0xF000 == header_data_.mid) {
        int size = static_cast<int>(sizeof(PLC_NAV_MSG));
        if (!flag_plc_send_encoder_fix_) {  // plc do not send encoder and fix data
            size -= (size_encoder_data_ + size_fix_data_);
        }
        if (size != len) {
            ROS_ERROR("R_err_6: size of PLC_NAV_MSG is [%d] not equal data length [%d]", size, len);
            return false;
        }
        memcpy((char*)(&nav_data_plc_), buffer, len);
        // pub plc imu data
        PublishImuData(nav_data_plc_);
        if (flag_plc_send_encoder_fix_) {
            PublishEncoderData(nav_data_plc_);
        }
        // pub nav res
        common_msgs::plc_res_nav_data res_data;
        res_data.capacity_remain = nav_data_plc_.capacity_remain;//剩余电量
        res_data.voltage = 0.001 * nav_data_plc_.voltage;//电压
        res_data.vel_linear = 0.001 * nav_data_plc_.v_x;//线速度
        res_data.status_sensor_signal = nav_data_plc_.status_sensor_signal;//传感器信号
        res_data.status_stop = nav_data_plc_.status_stop;//是否停止
        res_data.status_charge = nav_data_plc_.status_charge;//是否充电
        res_data.id_stop_node = nav_data_plc_.id_stop_node;//停止节点
        res_data.id_floor = nav_data_plc_.id_floor;//楼层号
        pub_res_data_.publish(res_data);
        // pub vel
        geometry_msgs::Twist vel_data;
        vel_data.linear.x = 0.001 * nav_data_plc_.v_x;  // PLC 上传的 中心 Vx线速度
        vel_data.linear.y = 0.001 * nav_data_plc_.v_y;  // PLC 上传的 中心 Vy线速度
        vel_data.angular.z = 0.001 * nav_data_plc_.w;   // PLC 上传的 中心 w角速度
        pub_vel_.publish(vel_data);
        // pub odom and tf
        PublishOdomDataAndTf(nav_data_plc_);
    } else {
        ROS_ERROR("R_err_5: mid not match ");
        return false;
    }
    return true;
}

bool PlcInteraction::HandleRecvActionMsg(const unsigned char* buffer, const int len)
{
    if (0xF000 == header_data_.mid) {
        const int size = (int)sizeof(PLC_ACTION_MSG);
        if (size > len) {
            ROS_ERROR("R_err_6: spe complete msg reply data len error (%d > %d)", size, len);
            return false;
        }
        memcpy((char*)(&action_data_from_plc_), buffer, len);

        // for (size_t i = 0; i < __MIO_PROTO_ACTUATOR_NUM__; ++i) {
        //     ROS_INFO(" action[%d] -> status: [%d], error_code: [%d], value: [%d] ", i,
        //     action_data_from_plc_.status_work[i].status_action,
        //                       action_data_from_plc_.status_work[i].error_code_action,
        //                       action_data_from_plc_.status_work[i].value_action);
        // }

        // 目前只是把动作机构1数据发出来了，因为平衡重只有一个货叉
        common_msgs::plc_res_action_data action_data;
        action_data.status_action = action_data_from_plc_.status_work[0].status_action;
        action_data.error_code_action = action_data_from_plc_.status_work[0].error_code_action;
        action_data.value_action = action_data_from_plc_.status_work[0].value_action;
        action_data.id_finish_action = action_data_from_plc_.status_work[0].id_action;

        pub_res_action_data_.publish(action_data);

    } else {
        ROS_ERROR("R_err_55: mid not match ");
        return false;
    }
    return true;
}

PlcInteraction::~PlcInteraction()
{
    m_stop_recv_nav_ = true;
    DELETE_THREAD(thread_recv_nav_);

    m_stop_recv_action_ = true;
    DELETE_THREAD(thread_recv_action_);
}
}  // namespace mio