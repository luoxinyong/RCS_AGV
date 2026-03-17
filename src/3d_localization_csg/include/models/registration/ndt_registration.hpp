#ifndef SLAM_AD_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define SLAM_AD_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include "pcl/registration/ndt.h"
#include "models/registration/registration_interface.hpp"
#include <chrono>

namespace h_x {
class NDTRegistration: public RegistrationInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(const float& res, const float& step_size, const float& trans_eps, const int& max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target,
                        const pcl::KdTreeFLANN<CloudData::POINT>::Ptr& input_kdtree = nullptr) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
    void GetIterCount(int& iter_count, float& costtime_scan_match) override;
  private:
    bool SetRegistrationParam(const float& res, const float& step_size, const float& trans_eps, const int& max_iter);

  private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}

#endif