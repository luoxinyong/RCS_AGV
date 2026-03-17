#ifndef SLAM_AD_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "sensor_data/pose_data.hpp"

namespace h_x {
class OdometrySubscriber {
    public:
        OdometrySubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size);
        OdometrySubscriber() = default;
        void ParseData(std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& deque_pose_data);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<PoseData, Eigen::aligned_allocator<PoseData>> new_pose_data_;
        std::mutex buff_mutex_;

        void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
};
}

#endif