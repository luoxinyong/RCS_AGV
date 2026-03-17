#ifndef SLAM_AD_PUBLISHER_TF_BROADCASTER_HPP_
#define SLAM_AD_PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace h_x {
class TFBroadCaster {
  public:
    TFBroadCaster(const std::string& frame_id, const std::string& child_frame_id);
    TFBroadCaster() = default;
    void SendTransform(Eigen::Matrix4f& pose, const double& time);
  protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
}
#endif