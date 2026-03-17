#include "models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace h_x {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(const float& res, const float& step_size, const float& trans_eps, const int& max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(const float& res, const float& step_size, const float& trans_eps, const int& max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);

    float trans_eps_final = trans_eps;
    int pcl_version = (PCL_VERSION-100000)/100;
    std::cout << " pcl_version: " << pcl_version << std::endl;
    if(pcl_version > 8) { //   PCL_VERSION >= PCL_VERSION_CALC(1, 8, 0)
        trans_eps_final = trans_eps*trans_eps;
        ndt_ptr_->setTransformationEpsilon(trans_eps_final);
    }
    
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps_final << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target,
                                     const pcl::KdTreeFLANN<CloudData::POINT>::Ptr& input_kdtree) {
    ndt_ptr_->setInputTarget(input_target);
    // LOG(INFO) << " SetInputTarget: target points size: " << input_target->points.size() << std::endl;
    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    //    LOG(INFO) << " ScanMatch, 111111 " << std::endl;
    
    ndt_ptr_->setInputSource(input_source);
    auto start_time = std::chrono::steady_clock::now();
    //    LOG(INFO) << " ScanMatch, 222222 " << std::endl;
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
     auto end_time = std::chrono::steady_clock::now();
    float cost_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    std::cout << "ScanMatch cost time: " << cost_time << " ms" << std::endl;
    //    LOG(INFO) << " ScanMatch, 333333 " << std::endl;
    result_pose = ndt_ptr_->getFinalTransformation();
       
    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}

void NDTRegistration::GetIterCount(int& iter_count, float& costtime_scan_match) {
   iter_count = 0;
   costtime_scan_match = 0;
}
}