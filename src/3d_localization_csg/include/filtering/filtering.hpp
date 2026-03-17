#ifndef SLAM_AD_LOCALIZATION_FILTERING_HPP_
#define SLAM_AD_LOCALIZATION_FILTERING_HPP_

#include <deque>
#include <string>
#include <unordered_map>
#include <map>
#include <atomic>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/pos_vel_data.hpp"
#include "sensor_data/pose_data.hpp"

#include "models/cloud_filter/box_filter.hpp"
#include "models/cloud_filter/cloud_filter_interface.hpp"
#include "models/registration/registration_interface.hpp"
#include "models/kalman_filter/kalman_filter.hpp"
#include "tools/common.h"


namespace h_x {
class Filtering {
 public:
  Filtering();

  bool Init(const CloudData &init_scan, const Eigen::Matrix4f &init_pose,
            const Eigen::Vector3f &init_vel, const IMUData &init_imu_data);
  bool InitRelocalization(
      std::shared_ptr<RegistrationInterface> &registration_ptr,
      const YAML::Node &config_node);
  bool Relocalization(const CloudData::CLOUD_PTR &current_scan_ptr,
                      Eigen::Matrix4f &relocalization_pose);
  bool RotationSearch(const CloudData::CLOUD_PTR &current_scan_ptr,
                      Eigen::Matrix4f &relocalization_pose);
  bool TranslationSearch(const CloudData::CLOUD_PTR &current_scan_ptr,
                      Eigen::Matrix4f &relocalization_pose);


  bool Update(const IMUData &imu_data);
  bool Correct(const int num_local_map, const IMUData &imu_data, const CloudData &cloud_data,
               const PosVelData &pos_vel_data, Eigen::Matrix4f &cloud_pose);

  // getters:
  bool HasInited() const { return has_inited_; }
  bool HasNewGlobalMap() const { return has_new_global_map_; }
  bool HasNewLocalMap() const { return has_new_local_map_; }

  void GetGlobalMap(CloudData::CLOUD_PTR &global_map);
  CloudData::CLOUD_PTR &GetLocalMap() { return local_map_ptr_; }
  CloudData::CLOUD_PTR &GetCurrentScan() { return current_scan_ptr_; }
  CloudData removePointsByRange(const CloudData &cloud_data, const double &min_range, const double &max_range);

  double GetTime(void) { return kalman_filter_ptr_->GetTime(); }
  Eigen::Matrix4f GetPose(void) { return current_pose_; }
  Eigen::Vector3f GetVel(void) { return current_vel_; }
  void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);
//   void SetSubmapData(const std::vector<SubmapOrigin>& submap_data);

  bool LoadSubmapData();
  void ShowSubmapData();
  void GetNearestSubmapId(const SubmapOrigin &current_pose, std::vector<SubmapOrigin>& nearest_submap, const int& count);
  // bool SetSubmap(const SubmapOrigin &submap_data);
  // bool GetSubmap(SubmapOrigin &submap_data);
  bool InitSubMap(float x, float y, float z);
  void GetScanMatchResult(int& iter_count, float& socre, float& costtime_scan_match);
  bool LoadSubMap(const int& id_submap, CloudData::CLOUD_PTR& submap_ptr);

 private:
  bool InitWithConfig(void);
  // a. filter initializer:
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface> &filter_ptr,
                  const YAML::Node &config_node);
  bool InitLocalMapSegmenter(const YAML::Node &config_node);
  bool InitFilters(const YAML::Node &config_node);
  // b. map initializer:
  bool InitGlobalMap(const YAML::Node &config_node);
  // d. frontend initializer:
  bool InitRegistration(
      std::shared_ptr<RegistrationInterface> &registration_ptr,
      const YAML::Node &config_node);
  // e. IMU-lidar fusion initializer:
  bool InitFusion(const YAML::Node &config_node);

  // local map setter:
  void SetFixPose(Eigen::Matrix4f &pose, float new_x, float new_y, float new_yaw);
  bool SwitchLocalMap(const int num_local_map, Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &cloud_pose);

  // init pose setter:
  bool SetInitGNSS(const Eigen::Matrix4f &init_pose);
  bool SetInitPose(const Eigen::Matrix4f &init_pose);

 private:
  std::string map_folder_path_ = "";
  std::string scan_context_path_ = "";

  std::string loop_closure_method_ = "";
  std::vector<SubmapOrigin> submap_data_;
  std::atomic_int32_t submap_id_global_;
  std::mutex lock_;

  // a. global map:
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
  // b. local map:
  std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  // c. current scan:
  std::shared_ptr<CloudFilterInterface> current_scan_filter_ptr_;

  // frontend:
  std::shared_ptr<RegistrationInterface> registration_ptr_;
  std::shared_ptr<RegistrationInterface> relocalization_match_ptr_;
  // IMU-lidar Kalman filter:
  struct {
    std::string FUSION_METHOD;

    std::unordered_map<std::string, KalmanFilter::MeasurementType>
        FUSION_STRATEGY_ID;
    KalmanFilter::MeasurementType FUSION_STRATEGY;
  } CONFIG;
  std::shared_ptr<KalmanFilter> kalman_filter_ptr_;
  KalmanFilter::Measurement current_measurement_;

  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;
  std::vector<int> vec_arround_submap_id_;
  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_submap_;
  std::map<int,CloudData::CLOUD_PTR> map_around_submap_ptr_;
  std::map<int, pcl::KdTreeFLANN<CloudData::POINT>::Ptr> map_around_submap_kdtree_ptr_;

  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();

  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
  double fitness_score_threshold_{0.2};
  int range_search_init_{2};
  int range_step_init_{1};
  int num_sample_init_{36};

  int iter_count_{0};
  float costtime_scan_match_{0.0};
  float fitness_socre_{0.0};
  int count_around_submap_{8};

  // switch map
  int count_hold_ = 0;
  int count_max_hold_ = 100;
  bool flag_start_hold_ = false;
  float x_hold_ = 0;
  float y_hold_ = 0;
  float theta_hold_ = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif