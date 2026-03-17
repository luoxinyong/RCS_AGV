#include "models/registration/small_gicp_registration.hpp"
#include <small_gicp/registration/registration_helper.hpp>

namespace h_x {

SmallGicpRegistration::SmallGicpRegistration(const YAML::Node& node) {
    
    int thread_nums = node["thread_nums"].as<int>();
    double threshlod = node["threshlod"].as<double>();//max_correspondence_distance
    float downsampling_resolution = node["downsampling_resolution"].as<float>();

    SetRegistrationParam(thread_nums, threshlod, downsampling_resolution);
}

SmallGicpRegistration::SmallGicpRegistration(const int& thread_nums, const double& threshlod, const float& downsampling_resolution){
        SetRegistrationParam(thread_nums, threshlod, downsampling_resolution);
}

bool SmallGicpRegistration::SetRegistrationParam(const int& thread_nums, const double& threshlod, const float& downsampling_resolution){
    
    setting.num_threads = thread_nums;
    setting.downsampling_resolution = downsampling_resolution;
    setting.max_correspondence_distance = threshlod;
    // setting.type = RegistrationSetting::VGICP;
    return true;
}  
std::vector<Eigen::Vector4f> SmallGicpRegistration::ConvertToEigenPointCloud(const CloudData::CLOUD_PTR& cloud) {
    std::vector<Eigen::Vector4f> points;
    // auto start_time = std::chrono::steady_clock::now();
    
    if (!cloud) {
        throw std::invalid_argument("Input cloud pointer is null");
    }
    
    if (cloud->empty()) {
        return points;
    }
    
    points.reserve(cloud->size());
    for (const auto& point : *cloud) {
        points.emplace_back(Eigen::Vector4f(point.x, point.y, point.z, 1.0f));
    }
    
    // auto end_time = std::chrono::steady_clock::now();
    // float cost_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    // std::cout << "ConvertToEigenPointCloud cost time: " << cost_time << " ms" << std::endl;
    return points;
}

void SmallGicpRegistration::SetInputTarget_(const CloudData::CLOUD_PTR& input_target){
    // 转换点云格式并降采样
    std::vector<Eigen::Vector4f> target = ConvertToEigenPointCloud(input_target);
    target_points_ = target;
}

bool SmallGicpRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target,
                        const pcl::KdTreeFLANN<CloudData::POINT>::Ptr& input_kdtree ){
    SetInputTarget_(input_target);
    return true;
}

bool SmallGicpRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose){
     auto start_time = std::chrono::steady_clock::now();
    
    // 转换源点云并降采样
    std::vector<Eigen::Vector4f> source = ConvertToEigenPointCloud(input_source);
    
    // // 初始变换矩阵
    // Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
    // init_T_target_source.matrix() = predict_pose.cast<double>();
    Eigen::Isometry3d predict_pose_iso = Eigen::Isometry3d::Identity();
    Eigen::Matrix4d predict_pose_d = predict_pose.cast<double>();
    predict_pose_iso.linear() = predict_pose_d.block<3, 3>(0, 0);      // 设置旋转部分
    predict_pose_iso.translation() = predict_pose_d.block<3, 1>(0, 3); // 设置平移部分
    // 执行配准
    last_result_ = align<float,4>(target_points_, source, predict_pose_iso, setting);

    // 更新结果
    result_pose = last_result_.T_target_source.matrix().cast<float>();
    
    // 转换并填充结果点云
    result_cloud_ptr->clear();
    result_cloud_ptr->reserve(source.size());
    
    // Eigen::Matrix4f T = result_pose;
    // for (const auto& pt : source) {
    //     Eigen::Vector4f transformed = T * pt.cast<float>();
    //     CloudData::POINT point;
    //     point.x = transformed.x();
    //     point.y = transformed.y();
    //     point.z = transformed.z();
    //     result_cloud_ptr->push_back(point);
    // }
    
    // 计算耗时
    auto end_time = std::chrono::steady_clock::now();
    float cost_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    std::cout << "ScanMatch cost time: " << cost_time << " ms" << std::endl;
    return last_result_.converged;
    // result_cloud_ptr = input_source;
}
float SmallGicpRegistration::GetFitnessScore(){
    // inlier_count / (1.0 + error)
    // last_result_.num_inliers/(1+last_result_.error)
    return (last_result_.error/last_result_.num_inliers);
}
void SmallGicpRegistration::GetIterCount(int& iter_count, float& costtime_scan_match){
    iter_count = last_result_.iterations;
}
// 显式实例化模板

}