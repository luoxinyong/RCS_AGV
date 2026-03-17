#ifndef SLAM_AD_MODELS_REGISTRATION_INTERFACE_HPP_
#define SLAM_AD_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "sensor_data/cloud_data.hpp"

namespace h_x {
class RegistrationInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target,
                                const pcl::KdTreeFLANN<CloudData::POINT>::Ptr& input_kdtree = nullptr) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;
    virtual void GetIterCount(int& iter_count, float& costtime_scan_match) = 0;
};
} 

#endif