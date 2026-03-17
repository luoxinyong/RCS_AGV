#ifndef SLAM_AD_LOCALIZATION_FILTERING_FLOW_HPP_
#define SLAM_AD_LOCALIZATION_FILTERING_FLOW_HPP_

#include <ostream>
#include <mutex>
#include <ctime>
#include <chrono>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "glog/logging.h"
#include "subscriber/int_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/pos_vel_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "publisher/point_publisher.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"

#include "filtering/filtering.hpp"
#include "scan_context/Scancontext.h"
#include "tools/common.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <string>
#include <ctime>
#include <sstream>
#include <fstream>
#include <mutex>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace h_x {
class FilteringFlow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  FilteringFlow(ros::NodeHandle &nh);
  bool Run();

private:
  bool LoadSubmapTable(const std::string& path_submap);
  bool ReadData();
  bool HasInited();

  bool HasData();
  //检查IMU原始数据raw的有效性：当前 IMU 数据的时间戳与滤波时间戳的差值是否 ≤ 0.02 秒（有需要同步的）
  bool HasIMUData(void) const {
    if (!imu_raw_data_buff_.empty()) {
      double diff_filter_time = current_imu_raw_data_.time - filtering_ptr_->GetTime();
      if (diff_filter_time <= 0.02) {
        return true;
      }
    }

    return false;
  }
  //如果 所有雷达、同步后IMU、位置速度三个缓冲区均非空 → 返回 true
  bool HasLidarData(void) const {
    // if(cloud_data_buff_.empty()) {
    //   LOG(INFO) << " cloud_data_buff_ is empty " << std::endl;
    // }
    // if(imu_synced_data_buff_.empty()) {
    //   LOG(INFO) << " imu_synced_data_buff_ is empty " << std::endl;
    // }
    // if(pos_vel_data_buff_.empty()) {
    //   LOG(INFO) << " pos_vel_data_buff_ is empty " << std::endl;
    // }
    
    return (!cloud_data_buff_.empty() && !imu_synced_data_buff_.empty() &&
            !pos_vel_data_buff_.empty());
  }
  bool HasIMUComesFirst(void) const {
    return imu_raw_data_buff_.front().time < cloud_data_buff_.front().time;
  }

  bool ValidIMUData();
  bool ValidLidarData();

  bool InitCalibration();
  bool InitLocalization();

  bool UpdateLocalization();
  bool CorrectLocalization(const int num_local_map);

  bool PublishGlobalMap();
  bool PublishLocalMap();
  bool PublishLidarOdom();
  bool PublishFusionOdom();

  bool UpdateOdometry(const double &time);

  bool setScanContextPose(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Matrix4f &init_pose);
  Eigen::MatrixXd loadSCD( const std::string file_path);

  std::vector<Eigen::MatrixXd> loadSCDS(std::string dir_path);

  void loadSCPose(const std::string& file_path, std::vector<Eigen::MatrixXd>&CloudKeyPoses);
  /**
   * @brief  save pose in KITTI format for evo evaluation
   * @param  pose, input pose
   * @param  ofs, output file stream
   * @return true if success otherwise false
   */
  bool SavePose(const Eigen::Matrix4f &pose, std::ofstream &ofs);
  void InitLocWithConfig();
  // void HandleLocalMap();
  void ShowSubmapData();

private:
   std::chrono::time_point<std::chrono::system_clock> time_start_;
   std::chrono::time_point<std::chrono::system_clock> time_pub_map_;
   std::chrono::time_point<std::chrono::system_clock> time_read_data_;
   std::chrono::time_point<std::chrono::system_clock> time_while_;
   std::chrono::time_point<std::chrono::system_clock> time_correct_start_;
   std::chrono::time_point<std::chrono::system_clock> time_correct_end_;
   std::chrono::time_point<std::chrono::system_clock> time_predict_;
   std::chrono::time_point<std::chrono::system_clock> time_s1_;
   std::chrono::time_point<std::chrono::system_clock> time_s2_;
   std::chrono::time_point<std::chrono::system_clock> time_wait_imu_lidar_;
   std::chrono::duration<double> elapsed_seconds_;

   std::chrono::time_point<std::chrono::system_clock> pose_init_time_;
   std::chrono::time_point<std::chrono::system_clock> pose_init_end_time;
   std::chrono::duration<double> time_pose_init_cost_;

   double costtime_pub_map_;
   double costtime_read_data_;
   double costtime_while_;
   double costtime_correct_;
   double costtime_predict_;
   double costtime_s1_;
   double costtime_s2_;
   double costtime_wait_imu_lidar_;
   size_t count_update_correct_failed_;
  //  std::chrono::time_point<std::chrono::system_clock> time_pub_map;
  //  std::chrono::time_point<std::chrono::system_clock> time_pub_map;
  // subscriber:
    // adding the SCManager 
  std::shared_ptr<SCManager> sc_manager_ptr_;
  // floor num raw:
  std::shared_ptr<IntSubscriber> floor_raw_sub_ptr_;
  int32_t floor_num_current_ = 0;
  // a. IMU raw:
  std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
  std::deque<IMUData,Eigen::aligned_allocator<IMUData>> imu_raw_data_buff_;
  // b. lidar:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::deque<CloudData,Eigen::aligned_allocator<CloudData>> cloud_data_buff_;
  // c. synced GNSS-odo measurement:
  std::shared_ptr<PosVelSubscriber> pos_vel_sub_ptr_;
  std::deque<PosVelData,Eigen::aligned_allocator<PosVelData>> pos_vel_data_buff_;
  // c. IMU synced:
  std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_;
  std::deque<IMUData,Eigen::aligned_allocator<IMUData>> imu_synced_data_buff_;
  // e. GNSS:
  std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
  std::deque<PoseData,Eigen::aligned_allocator<PoseData>> gnss_data_buff_;
  // f. lidar to imu tf:
  std::shared_ptr<TFListener> base_to_lidar_ptr_;
  Eigen::Matrix4f base_to_laser_ = Eigen::Matrix4f::Identity();

  // publisher:
  // a. global-local map and current scan:
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
  // b. odometry:
  std::shared_ptr<PointPublisher> time_cost_exter_pub_ptr_;
  std::shared_ptr<PointPublisher> time_cost_while_pub_ptr_;
  std::shared_ptr<PointPublisher> time_wait_imu_lidar_ptr_;
  std::shared_ptr<PointPublisher> count_fail_pub_ptr_;
  std::shared_ptr<PointPublisher> result_scan_match_pub_ptr_;
  std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> ref_odom_pub_ptr_;
  // c. tf:
  std::shared_ptr<TFBroadCaster> base_tf_pub_ptr_;

  // filtering instance:
  std::shared_ptr<Filtering> filtering_ptr_;

  IMUData current_imu_raw_data_;
  CloudData current_cloud_data_;
  IMUData current_imu_synced_data_;
  PosVelData current_pos_vel_data_;
  PoseData current_gnss_data_;

  // lidar odometry frame in map frame:
  geometry_msgs::PointStamped point_;
  geometry_msgs::PointStamped data_time_while_;
  geometry_msgs::PointStamped data_time_wait_imu_lidar_;
  geometry_msgs::PointStamped count_failed_pose_;
  geometry_msgs::PointStamped result_scan_match_;
  Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();
  Eigen::Matrix4f laser_pose_ = Eigen::Matrix4f::Identity();
  std::vector<Eigen::MatrixXd> copy_CloudKeyPoses;
  // The scancontext configuration from the yaml file.
  std::string scan_context_file_path_;
  bool use_sc_loc_init_ = true;
  double xx_ = 0;
  double yy_ = 0;
  double zz_ = 0;
  int count_jump_ = 0;

  bool use_file_pose_init_ = true;
  std::string file_pose_init_path_;

  bool  use_gnss_loc_init_ = true;
  double x_fix_init_pose_{0.0};
  double y_fix_init_pose_{0.0};
  double z_fix_init_pose_{0.0};
  double yaw_fix_init_pose_{0.0};
  // trajectory for evo evaluation:
  struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t N = 0;
    std::deque<double> time_;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> fused_;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> lidar_;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> ref_;
  } trajectory;
};
}

#endif