#ifndef SLAM_AD_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <ros/ros.h>

#include "sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"

namespace h_x {
class IMUSubscriber {
 public:
  IMUSubscriber(ros::NodeHandle& nh, const std::string& topic_name,
                const size_t& buff_size);
  IMUSubscriber() = default;
  void ParseData(std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& deque_imu_data);

 private:
  void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> new_imu_data_;
  std::mutex buff_mutex_;
};
}  // namespace h_x
#endif