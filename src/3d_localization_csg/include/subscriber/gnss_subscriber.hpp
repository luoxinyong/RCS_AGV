
#ifndef SLAM_AD_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <ros/ros.h>

#include "sensor_data/gnss_data.hpp"
#include "sensor_msgs/NavSatFix.h"

namespace h_x {
class GNSSSubscriber {
 public:
  GNSSSubscriber(ros::NodeHandle& nh, const std::string& topic_name,
                 const size_t& buff_size);
  GNSSSubscriber() = default;
  void ParseData(std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& deque_gnss_data);

 private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>> new_gnss_data_;
  std::mutex buff_mutex_;
};
}  // namespace h_x
#endif