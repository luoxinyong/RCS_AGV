#include "models/registration/small_gicp_registration.hpp"
#include "models/registration/ndt_registration.hpp"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Dense>
#include <cmath>

using namespace small_gicp;
// 计算旋转矩阵之间的角度误差（弧度）
double computeRotationError(const Eigen::Matrix4f& estimated, const Eigen::Matrix4f& groundtruth) {
    Eigen::Matrix3f R_est = estimated.block<3, 3>(0, 0);
    Eigen::Matrix3f R_gt = groundtruth.block<3, 3>(0, 0);
    
    // 计算相对旋转矩阵
    Eigen::Matrix3f R_error = R_est * R_gt.transpose();
    
    // 从旋转矩阵提取旋转角度
     float trace = R_error.trace();
    
    // 计算角度值并处理数值精度问题
    float value = (trace - 1.0f) / 2.0f;
    value = std::max(-1.0f, std::min(1.0f, value)); // 限制在[-1, 1]范围内
    
    // 计算角度（弧度）
    return std::acos(value);
}

// 计算旋转误差（度）
double computeRotationErrorDegrees(const Eigen::Matrix4f& estimated, const Eigen::Matrix4f& groundtruth) {
    return computeRotationError(estimated, groundtruth) * 180.0 / M_PI;
}

double computeTranslationError(const Eigen::Matrix4f& estimated, const Eigen::Matrix4f& groundtruth) {
    Eigen::Vector3f t_est = estimated.block<3, 1>(0, 3);
    Eigen::Vector3f t_gt = groundtruth.block<3, 1>(0, 3);
    
    return (t_est - t_gt).norm();
}

// }
int main(int argc, char *argv[]){
    ros::init(argc, argv, "ndt_test");
    ros::NodeHandle nh;
    std::cout << "ndt: "  << std::endl;
    pcl::PLYReader reader;
    h_x::NDTRegistration Registration(0.25,0.5,0.01,30);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_target = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f groundtruth = Eigen::Matrix4f::Identity();
    groundtruth.block<3, 3>(0, 0) << 0.999925,0.0121483,0,-0.0121523,0.999924,0,0,0,1;
    groundtruth.block<3, 1>(0, 3) << 0.488882,0.121214,-0.0253342;
    // groundtruth.block<3, 3>(0, 0) << 0.999925,0.0121483,-0.00177009,-0.0121523,0.999924,-0.00228657,0.00174218,0.00230791,0.999996;
    // groundtruth.block<3, 1>(0, 3) << 0.488882,0.121214,-0.0253342;
    // reader.read("src/3d_localization_csg/src/models/data/target.ply", *raw_target);
   
    reader.read("src/3d_localization_csg/src/models/data/source.ply", *raw_source);
    // reader.read("src/3d_localization_csg/src/models/data/target.ply", *raw_target);
    pcl::transformPointCloud(*raw_source, *raw_target,groundtruth);
    Registration.SetInputTarget(raw_target);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Registration.ScanMatch(raw_source, Eigen::Matrix4f::Identity(), result_cloud, result_pose);
    // std::cout << "fitnessSocre: \n" << Registration.GetFitnessScore() << std::endl;
    std::cout << "result_pose: \n" << result_pose << std::endl;
    std::cout << "NDT finessScore: \n" << Registration.GetFitnessScore() << std::endl;


    std::cout << "small_gicp: "<< std::endl;
    h_x::SmallGicpRegistration Registration_small(8,1.0,0.25);
  //   std::vector<Eigen::Vector4f> target_points = read_ply("src/3d_localization_csg/src/models/data/target.ply");
  //   std::vector<Eigen::Vector4f> source_points = read_ply("src/3d_localization_csg/src/models/data/source.ply");
  //   if (target_points.empty() || source_points.empty()) {
  //     std::cerr << "error: failed to read points from data/(target|source).ply" << std::endl;
  //     return 1;
  //   }

  // const auto convert_to_pcl = [](const std::vector<Eigen::Vector4f>& raw_points) {
  //   std::cout<<"in here..."<<std::endl;
  //   auto points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  //   points->resize(raw_points.size());
  //   for (size_t i = 0; i < raw_points.size(); i++) {
  //     points->at(i).getVector4fMap() = raw_points[i];
  //   }
  //   return points;
  // };

  // pcl::PointCloud<pcl::PointXYZ>::Ptr raw_target = convert_to_pcl(target_points);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr raw_source = convert_to_pcl(source_points);

  Registration_small.SetInputTarget(raw_target);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f result_pose_ = Eigen::Matrix4f::Identity();
    
    // 执行配准
    Registration_small.ScanMatch(raw_source, Eigen::Matrix4f::Identity(), result_cloud_, result_pose_);
    
    std::cout << "result_pose: \n" << result_pose_ << std::endl;
    std::cout << "small fitnessSocre: \n" << Registration_small.GetFitnessScore() << std::endl;

  
  std::cout << "NDT rotation error: \n" << computeRotationErrorDegrees(result_pose, groundtruth) << std::endl;
  std::cout << "NDT translation error: \n" << computeTranslationError(result_pose, groundtruth) << std::endl;

  std::cout << "small rotation error: \n" << computeRotationErrorDegrees(result_pose_, groundtruth) << std::endl;
  std::cout << "small translation error: \n" << computeTranslationError(result_pose_, groundtruth) << std::endl;

}