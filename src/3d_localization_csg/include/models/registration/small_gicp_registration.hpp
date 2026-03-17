#ifndef SLAM_AD_MODELS_REGISTRATION_SMALLGICP_REGISTRATION_HPP_
#define SLAM_AD_MODELS_REGISTRATION_SMALLGICP_REGISTRATION_HPP_

#include "models/registration/registration_interface.hpp"
#include <Eigen/Dense>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include <vector>
#include <chrono>

using namespace small_gicp;

namespace h_x {
class SmallGicpRegistration : public RegistrationInterface {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SmallGicpRegistration(const YAML::Node& node);
        SmallGicpRegistration(const int& thread_nums, const double& threshlod, const float& downsampling_resolution);
        void SetInputTarget_(const CloudData::CLOUD_PTR& input_target);
        bool SetInputTarget(const CloudData::CLOUD_PTR& input_target,
                        const pcl::KdTreeFLANN<CloudData::POINT>::Ptr& input_kdtree = nullptr) override;
        bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
        float GetFitnessScore() override;
    void GetIterCount(int& iter_count, float& costtime_scan_match) override;
    private:
        bool SetRegistrationParam(const int& thread_nums,  const double& threshlod, const float& downsampling_resolution);
        // template <typename T>
        // bool isPointCloudAligned(const typename pcl::PointCloud<T>::Ptr &cloud);
        std::vector<Eigen::Vector4f> ConvertToEigenPointCloud(const CloudData::CLOUD_PTR& cloud);
        RegistrationSetting setting;
        std::vector<Eigen::Vector4f> target_points_;
        RegistrationResult last_result_;
};
}
#endif