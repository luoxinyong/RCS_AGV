#ifndef SLAM_AD_TF_LISTENER_HPP_
#define SLAM_AD_TF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace h_x {
class TFListener {
 public:
  TFListener(ros::NodeHandle& nh, const std::string& base_frame_id,
             const std::string& child_frame_id);
  TFListener() = default;

  bool LookupData(Eigen::Matrix4f& transform_matrix);

 private:
  bool TransformToMatrix(const tf::StampedTransform& transform,
                         Eigen::Matrix4f& transform_matrix);

 private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};
}  // namespace h_x

#endif