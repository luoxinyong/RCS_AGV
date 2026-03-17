#ifndef SLAM_AD_SENSOR_DATA_POSE_DATA_HPP_
#define SLAM_AD_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace h_x {

class PoseData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();

    Eigen::Quaternionf GetQuaternion();
};

}

#endif