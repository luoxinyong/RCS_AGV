#ifndef SLAM_AD_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensor_data/cloud_data.hpp"

namespace h_x {
class CloudSubscriber {
 public:
  CloudSubscriber(ros::NodeHandle& nh, const std::string& topic_name,
                  const size_t& buff_size);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData, Eigen::aligned_allocator<CloudData>>& deque_cloud_data);

 private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData, Eigen::aligned_allocator<CloudData>> new_cloud_data_;
  std::mutex buff_mutex_;
};

}  // namespace h_x

#endif