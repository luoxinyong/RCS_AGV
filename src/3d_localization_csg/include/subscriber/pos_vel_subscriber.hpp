#ifndef SLAM_AD_SUBSCRIBER_POSEVEL_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_POSEVEL_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "loc_ad/PosVel.h"
#include "sensor_data/pos_vel_data.hpp"

namespace h_x {
class PosVelSubscriber {
    public:
        PosVelSubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size);
        PosVelSubscriber() = default;
        void ParseData(std::deque<PosVelData, Eigen::aligned_allocator<PosVelData>>& pos_vel_data_buff);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<PosVelData, Eigen::aligned_allocator<PosVelData>> new_pos_vel_data_;

        std::mutex buff_mutex_; 
        void msg_callback(const loc_ad::PosVelConstPtr& pos_vel_msg_ptr);
};
}


#endif