#ifndef SLAM_AD_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define SLAM_AD_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensor_data/cloud_data.hpp"

namespace h_x {
class CloudPublisher {
 public:
  CloudPublisher(ros::NodeHandle& nh, const std::string& topic_name,
                 const std::string& frame_id, const size_t& buff_size);
  CloudPublisher() = default;
  void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);
  void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, const double& time);
  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
  void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, const ros::Time& time);
};
}  // namespace h_x
#endif