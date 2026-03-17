#ifndef SLAM_AD_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define SLAM_AD_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "models/scan_adjust/distortion_adjust.hpp"
#include "sensor_data/velocity_data.hpp"
#include "sensor_data/cloud_data.hpp"

namespace h_x {
class DistortionAdjust {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    void SetMotionInfo(const float& scan_period, const VelocityData& velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);
    void AdjustRsCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(const float& real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
}


#endif