#ifndef SLAM_AD_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_data/velocity_data.hpp"

namespace h_x {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>& deque_velocity_data);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>> new_velocity_data_;

    std::mutex buff_mutex_;
    void msg_callback(const nav_msgs::OdometryConstPtr& twist_msg_ptr);
};

}


#endif