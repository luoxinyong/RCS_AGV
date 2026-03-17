#include "subscriber/velocity_subscriber.hpp"
#include "glog/logging.h"

namespace h_x {
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& twist_msg_ptr) {
    buff_mutex_.lock();
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.twist.angular.z;

    new_velocity_data_.emplace_back(velocity_data);
    buff_mutex_.unlock();
}

void VelocitySubscriber::ParseData(std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>>& velocity_data_buff) {
    buff_mutex_.lock();
    if (new_velocity_data_.size() > 0) {
        velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
    buff_mutex_.unlock();
}
}