#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "filtering/filtering.hpp"
#include "global_defination/global_defination.h"
#include "models/cloud_filter/no_filter.hpp"
#include "models/cloud_filter/voxel_filter.hpp"
#include "models/registration/ndt_registration.hpp"
#include "models/registration/small_gicp_registration.hpp"
#include "models/kalman_filter/error_state_kalman_filter.hpp"

namespace h_x {

Filtering::Filtering()
    : global_map_ptr_(new CloudData::CLOUD()),
      local_map_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD()) {
  submap_id_global_ = 2;
  map_around_submap_ptr_.clear();
  vec_arround_submap_id_.clear();
  // load ROS config:
  InitWithConfig();
}

// bool Filtering::Init(const CloudData &init_scan,
//                           const Eigen::Vector3f &init_vel,
//                           const IMUData &init_imu_data) {
//   // TODO if use scan init, need add function

//   return false;
// }
// 打印已加载的子地图元数据，用于验证数据是否正确加载。
void Filtering::ShowSubmapData() {
  for (const auto& one_submap : submap_data_) {
    ROS_INFO("one origin is: [%f, %f, %f, %d]", one_submap.x, one_submap.y, one_submap.z, one_submap.index);
  }
}

// 加载子地图元数据
/** Load submap data to memory **/
bool Filtering::LoadSubmapData()
{
    std::string file_submap = map_folder_path_ + "/submap.csv";
    ROS_INFO("Loading table from %s", file_submap.c_str());
    std::ifstream ifs(file_submap);
    if (ifs) {
        while (!ifs.eof()) {
            std::string data;
            std::getline(ifs, data);
            if (data == "") {
                continue;
            }
            std::istringstream stream(data);
            std::string buffer;
            SubmapOrigin origin_submap;
            int index = 0;
            while (std::getline(stream, buffer, ',')) {
                index++;
                if (index == 1) {
                    origin_submap.x = std::stof(buffer);
                    continue;
                }
                if (index == 2) {
                    origin_submap.y = std::stof(buffer);
                    continue;
                }
                if (index == 3) {
                    origin_submap.z = std::stof(buffer);
                    continue;
                }
                if (index == 6) {
                    origin_submap.index = std::stof(buffer);
                    submap_data_.push_back(origin_submap);
                    continue;
                }
            }
        }
        ifs.close();
    } else {
        ROS_ERROR("can not open file: %s", file_submap.c_str());
        return false;
    }
    return true;
}

bool Filtering::Init(const CloudData& init_scan, const Eigen::Matrix4f &init_pose,
                          const Eigen::Vector3f &init_vel,
                          const IMUData &init_imu_data) {
  ROS_INFO(" RLM Init start: [%f, %f, %f]", init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));
  InitSubMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));
  ROS_INFO(" RLM Init end ......");
  relocalization_match_ptr_->SetInputTarget(local_map_ptr_);
  // LOG(INFO) << " Init 222222 " << std::endl;
  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  CloudData init_scan_narrowed = removePointsByRange(init_scan, 0, 60);
  current_scan_filter_ptr_->Filter(init_scan_narrowed.cloud_ptr, filtered_cloud_ptr);
  // LOG(INFO) << " Init 333333 " << std::endl;
  Eigen::Matrix4f relocalization_pose = init_pose;
  if(Relocalization(filtered_cloud_ptr, relocalization_pose)) {
     if (SetInitGNSS(relocalization_pose)) {
      current_vel_ = init_vel;
      kalman_filter_ptr_->Init(relocalization_pose.cast<double>(), current_vel_.cast<double>(), init_imu_data);
      return true;
    }
  }
  return false;
}

bool Filtering::Relocalization(const CloudData::CLOUD_PTR &current_scan_ptr, Eigen::Matrix4f& relocalization_pose) {
  if (TranslationSearch(current_scan_ptr, relocalization_pose)) {
    return true;
  } else {
    return false;
  }
}

//旋转搜索（转角）
bool Filtering::RotationSearch(const CloudData::CLOUD_PTR &current_scan_ptr, Eigen::Matrix4f& relocalization_pose) {
  std::multimap<double, Eigen::Matrix4f, std::less<double>, 
      Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4f>>> map_score_sort;//保存位姿，用分数-位姿对的形式存储
  // 1. 生成旋转候选集，以初始位姿为中心选择附近一定范围的区域
  map_score_sort.clear();
  // LOG(INFO) << " Relocalization 111111 " << std::endl;
  for (size_t i = 0; i < num_sample_init_; ++i) {
    double yaw_sample = (static_cast<double>(i * 2.0 * M_PI)) / (static_cast<double>(num_sample_init_));//360°范围采样
    if(yaw_sample > M_PI) {
      yaw_sample -= static_cast<double>(2.0 * M_PI);
    }
    Eigen::AngleAxisf guess_orientation(yaw_sample, Eigen::Vector3f::UnitZ());//生成旋转候选，绕Z轴旋转
    Eigen::Matrix4f guess_pose = Eigen::Matrix4f::Identity();
    guess_pose.block<3,3>(0,0) = guess_orientation.toRotationMatrix();
    guess_pose.block<3,1>(0,3) = relocalization_pose.block<3,1>(0,3);
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    
    // 2. 对每个旋转候选调用NDT匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
      //  LOG(INFO) << " Relocalization, for, 111111 " << std::endl;
    relocalization_match_ptr_->ScanMatch(current_scan_ptr, guess_pose, result_cloud_ptr, result_pose);
      //  LOG(INFO) << " Relocalization, for, 222222 " << std::endl;
    // 3. 计算每个旋转候选的匹配分数
    double score_fitness = relocalization_match_ptr_->GetFitnessScore();
    ROS_INFO(" one fitness score: %f ", score_fitness);
    map_score_sort.insert(std::make_pair(score_fitness,result_pose));
  }
  // 4. 选择最佳匹配NDT使用
  if(map_score_sort.size()) {
    if(map_score_sort.begin()->first < fitness_score_threshold_) {
      relocalization_pose = map_score_sort.begin()->second;
      ROS_INFO("Relocalization successd, fitness score: %f", map_score_sort.begin()->first);
      LOG(INFO) << "relocalization_pose is: " << relocalization_pose << std::endl;
      return true;
    }
  }
  // if(map_score_sort.size()) {
  //   if(map_score_sort.rbegin()->first > fitness_score_threshold_) {
  //     relocalization_pose = map_score_sort.rbegin()->second;
  //     ROS_INFO("Relocalization successd, fitness score: %f", map_score_sort.rbegin()->first);
  //     LOG(INFO) << "relocalization_pose is: " << relocalization_pose << std::endl;
  //     return true;
  //   }
  // }
  return false;
}

// 平移搜索（距离）
bool Filtering::TranslationSearch(const CloudData::CLOUD_PTR &current_scan_ptr, Eigen::Matrix4f& relocalization_pose) {
    Eigen::Matrix4f translation_pose = relocalization_pose;
    // 1、生成平移候选集，以初始位姿为中心选择附近一定范围的区域
    std::vector<std::pair<double, double>> match_search_space;
    match_search_space.push_back(std::make_pair(0, 0));
    for (int x = range_step_init_; x <= range_search_init_; x += range_step_init_) {
      for (int y = range_step_init_; y <= range_search_init_; y += range_step_init_) {
        match_search_space.push_back(std::make_pair(x, y));
        match_search_space.push_back(std::make_pair(x, -y));
        match_search_space.push_back(std::make_pair(-x, y));
        match_search_space.push_back(std::make_pair(-x, -y));
      }
    }
    // 2. 按距离排序（从近到远）
    std::sort(match_search_space.begin(), match_search_space.end(),
              [](const std::pair<double, double> &a, const std::pair<double, double> &b) {
                return a.first * a.first + a.second * a.second <
                       b.first * b.first + b.second * b.second;
              });

    ROS_INFO(" match_search_space size : %d", match_search_space.size());
    // 3. 对每个平移候选调用RotationSearch
    for (auto matcher : match_search_space) {
      ROS_INFO("matcher_x_y: [%f, %f] ", matcher.first, matcher.second);
      translation_pose(0, 3) = relocalization_pose(0, 3) + matcher.first;
      translation_pose(1, 3) = relocalization_pose(1, 3) + matcher.second;
      if (RotationSearch(current_scan_ptr, translation_pose)) {
        relocalization_pose = translation_pose;
        return true;
      }
    }
    return false;
}

bool Filtering::Update(const IMUData &imu_data) {
  if (kalman_filter_ptr_->Update(imu_data)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
    return true;
  }

  return false;
}

bool Filtering::Correct(const int num_local_map,
                             const IMUData &imu_data,
                             const CloudData &cloud_data,
                             const PosVelData &pos_vel_data,
                             Eigen::Matrix4f &cloud_pose
                             ) 
  {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr,
                               indices);

  // downsample:
  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

  // LOG(INFO) << " Correct -> laser, filter size: " << (*cloud_data.cloud_ptr).points.size() << ", " << filtered_cloud_ptr->points.size() << std::endl;

  if (!has_inited_) {
    predict_pose = current_gnss_pose_;
  }

  // matching:
  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
  // std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
  lock_.lock();
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr, cloud_pose);
  registration_ptr_->GetIterCount(iter_count_, costtime_scan_match_);
  fitness_socre_ = registration_ptr_->GetFitnessScore();
  lock_.unlock();

  SwitchLocalMap(num_local_map, predict_pose, cloud_pose);

  // std::cout << " predict: x: " << predict_pose(0,3) << " ,y: " << predict_pose(1,3) <<" ,z: " << predict_pose(2,3)
  //   << " correct: x: " << cloud_pose(0,3) << " ,y: " << cloud_pose(1,3) <<" ,z: " << cloud_pose(2,3)<<std::endl;

  // std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
  // std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  // double elapsed_ms = elapsed_seconds.count() * 1000;
  // LOG(INFO) << " cost time of match is: " << elapsed_ms;
  pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_,
                           cloud_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * cloud_pose;
  predict_pose = cloud_pose * step_pose;
  last_pose = cloud_pose;


  // set lidar measurement:
  current_measurement_.time = cloud_data.time;
  current_measurement_.T_nb =
      (init_pose_.inverse() * cloud_pose).cast<double>();
  current_measurement_.v_b = pos_vel_data.vel.cast<double>();
  current_measurement_.w_b =
      Eigen::Vector3d(imu_data.angular_velocity.x, imu_data.angular_velocity.y,
                      imu_data.angular_velocity.z);

  // Kalman correction:
  if (kalman_filter_ptr_->Correct(imu_data, KalmanFilter::MeasurementType::POSE_VEL,
                                  current_measurement_)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

    return true;
  }

  return false;
}

void Filtering::GetScanMatchResult(int& iter_count, float& socre, float& costtime_scan_match) {
  iter_count = iter_count_;
  costtime_scan_match = costtime_scan_match_;
  socre = fitness_socre_;
}

void Filtering::GetGlobalMap(CloudData::CLOUD_PTR &global_map) {
  // downsample global map for visualization:
  global_map_filter_ptr_->Filter(local_map_ptr_, global_map);
  has_new_global_map_ = false;
}

CloudData Filtering::removePointsByRange(const CloudData &cloud_data, const double& min_range, const double& max_range)
{
  CloudData narrowed_scan;
  narrowed_scan.time = cloud_data.time;

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for(auto point:cloud_data.cloud_ptr->points) {
    double square_distance = point.x * point.x + point.y * point.y;
    if(square_min_range <= square_distance && square_distance <= square_max_range) {
      narrowed_scan.cloud_ptr->points.push_back(point);
    }
  }
  return narrowed_scan;
}

void Filtering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
  pose = init_pose_ * current_pose_;
  vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
}

bool Filtering::InitWithConfig(void) {
  std::string config_file_path = ros::package::getPath("loc_ad") + "/config/filtering/filtering.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file_path);

  // a. init filters:
  InitFilters(config_node);
  // b. init map:
  InitGlobalMap(config_node);
  // c. init scan context manager:
  // d. init frontend:
  InitRegistration(registration_ptr_, config_node);
  //  init reloc:
  InitRelocalization(relocalization_match_ptr_, config_node);
  // e. init fusion:
  InitFusion(config_node);

  return true;
}

bool Filtering::InitFilter(
    std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr,
    const YAML::Node &config_node) {
  std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

  ROS_INFO(" Filter Method %s is %s", filter_mothod.c_str(), filter_user.c_str());

  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::shared_ptr<VoxelFilter>(new VoxelFilter(config_node[filter_mothod][filter_user]));
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::shared_ptr<NoFilter>(new NoFilter());
  } else {
    ROS_ERROR("Filter method %s not find %s", filter_mothod.c_str(), filter_user.c_str());
    return false;
  }

  return true;
}

bool Filtering::InitLocalMapSegmenter(const YAML::Node &config_node) {
  local_map_segmenter_ptr_ = std::shared_ptr<BoxFilter>(new BoxFilter(config_node));
  return true;
}

bool Filtering::InitFilters(const YAML::Node &config_node) {
  // a. global map filter -- downsample point cloud map for visualization:
  InitFilter("global_map", global_map_filter_ptr_, config_node);
  // b. local map filter -- downsample & ROI filtering for scan-map matching:
  InitLocalMapSegmenter(config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  // c. scan filter --
  InitFilter("current_scan", current_scan_filter_ptr_, config_node);

  return true;
}

bool Filtering::InitGlobalMap(const YAML::Node &config_node) {
  map_folder_path_ = config_node["map_path"].as<std::string>();
  int id_submap = 0;
  LoadSubMap(id_submap, global_map_ptr_);
  has_new_global_map_ = true;
  return true;
}

bool Filtering::InitRegistration(
    std::shared_ptr<RegistrationInterface> &registration_ptr,
    const YAML::Node &config_node) {
  std::string registration_method = config_node["registration_method"].as<std::string>();

  ROS_INFO(" Point Cloud Registration Method: %s", registration_method.c_str());

  if (registration_method == "NDT") {
    registration_ptr =
        std::shared_ptr<NDTRegistration>(new NDTRegistration(config_node[registration_method]));
  } 
  else if (registration_method == "Small_GICP") {
    registration_ptr =
        std::shared_ptr<SmallGicpRegistration>(new SmallGicpRegistration(config_node[registration_method]));
  } 
  
  else {
    ROS_ERROR("Registration method %s not find", registration_method.c_str());
    return false;
  }

  return true;
}


bool Filtering::InitRelocalization(
    std::shared_ptr<RegistrationInterface> &registration_ptr,
    const YAML::Node &config_node) {
  range_search_init_ = config_node["range_search_loc_init"].as<int>();
  range_step_init_   = config_node["range_step_loc_init"].as<int>();
  num_sample_init_   = config_node["num_sample_loc_init"].as<int>();
  std::string match_method_loc_init = config_node["match_method_loc_init"].as<std::string>();
  if (match_method_loc_init == "NDT_INIT") {
    registration_ptr = std::shared_ptr<NDTRegistration>(new NDTRegistration(config_node["NDT_INIT"]));
    fitness_score_threshold_ = config_node["NDT_INIT"]["fitness_score_threshold"].as<double>();
    ROS_INFO(" NDT_INIT, fitness_score_threshold: %f", fitness_score_threshold_);
  } 
  else if (match_method_loc_init == "Small_GICP_INIT") {
    registration_ptr = std::shared_ptr<SmallGicpRegistration>(new SmallGicpRegistration(config_node[match_method_loc_init]));
    fitness_score_threshold_ = config_node["Small_GICP_INIT"]["fitness_score_threshold"].as<double>();
    ROS_INFO(" Small_GICP_INIT, fitness_score_threshold: %f", fitness_score_threshold_);
  } 
  else {
    ROS_ERROR("Registration method %s not find", match_method_loc_init.c_str());
    return false;
  }

  return true;
}


bool Filtering::InitFusion(const YAML::Node &config_node) {
  // set up fusion method:
  CONFIG.FUSION_METHOD = config_node["fusion_method"].as<std::string>();
  if (CONFIG.FUSION_METHOD == "error_state_kalman_filter") {
    kalman_filter_ptr_ = std::shared_ptr<ErrorStateKalmanFilter>(new ErrorStateKalmanFilter(config_node[CONFIG.FUSION_METHOD]));
  } else {
    ROS_ERROR("Fusion method %s not find", CONFIG.FUSION_METHOD.c_str());
    return false;
  }
  ROS_INFO("Localization Fusion Method: %s", CONFIG.FUSION_METHOD.c_str());

  return true;
}

bool Filtering::SetInitGNSS(const Eigen::Matrix4f &gnss_pose) {
  static int gnss_cnt = 0;

  current_gnss_pose_ = gnss_pose;

  if (gnss_cnt == 0) {
    SetInitPose(gnss_pose);
    has_inited_ = true;
  } else if (gnss_cnt > 3) {
    has_inited_ = true;
  }
  gnss_cnt++;

  return true;
}

bool Filtering::SetInitPose(const Eigen::Matrix4f &init_pose) {
  init_pose_ = init_pose;
  return true;
}

void Filtering::GetNearestSubmapId(const SubmapOrigin &current_pose, std::vector<SubmapOrigin>& nearest_submaps, const int& count)
{
  SubmapOrigin one_nearest_submap;
  if(!submap_data_.size()) {
    ROS_ERROR(" not find submap data ...... ");
    return;
  }
  auto cmp = [&current_pose](SubmapOrigin firstPoint, SubmapOrigin secondPoint) -> bool {
      if( sqrt( pow(current_pose.x-firstPoint.x,2) + pow(current_pose.y-firstPoint.y,2) + pow(current_pose.z-firstPoint.z,2)) >= 
          sqrt( pow(current_pose.x-secondPoint.x,2)+ pow(current_pose.y-secondPoint.y,2)+ pow(current_pose.z-secondPoint.z,2)) ) {
          return true;
      } else {
          return false;
      }
  };
  SubmapHeap<SubmapOrigin> submap_heap{cmp};
  for (const auto& one_submap: submap_data_) {
      submap_heap.push(one_submap);
  }
  int count_near_submap = count;
  if(submap_data_.size() < count_near_submap) {
    count_near_submap = submap_data_.size();
  }
  for (int i = 0; i < count_near_submap; i++) {
      one_nearest_submap = submap_heap.top();
      nearest_submaps.push_back(one_nearest_submap); 
      submap_heap.pop();
  }
}

bool Filtering::LoadSubMap(const int& id_submap, CloudData::CLOUD_PTR& submap_ptr) {
  ROS_INFO(" LoadSubMap, submap id is: %d", id_submap);
  std::string path_submap = map_folder_path_ + "/" + std::to_string(id_submap) + ".pcd";
  pcl::io::loadPCDFile(path_submap, *submap_ptr);

  local_map_filter_ptr_->Filter(submap_ptr, submap_ptr);
  ROS_INFO(" LoadSubMap, finished, filtered local map, size: %d", submap_ptr->points.size());
  return true;
}

bool Filtering::InitSubMap(float x, float y, float z) {
  SubmapOrigin current;
  std::vector<SubmapOrigin> near_submap;
  near_submap.clear();
  current.x = x;
  current.y = y;
  current.z = z;
  // TODO: maybe submap count is only one
  GetNearestSubmapId(current, near_submap, count_around_submap_);
  CloudData::CLOUD_PTR  submap_ptr(new CloudData::CLOUD());
  for (int i = 0; i < near_submap.size(); ++i) {
      submap_ptr.reset(new CloudData::CLOUD());
      LoadSubMap(near_submap[i].index, submap_ptr);
      map_around_submap_ptr_.insert(std::pair<int,CloudData::CLOUD_PTR>(near_submap[i].index,submap_ptr));

      pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_from_submap(new pcl::KdTreeFLANN<CloudData::POINT>());
      kdtree_from_submap->setInputCloud(submap_ptr);
      map_around_submap_kdtree_ptr_.insert(
            std::pair<int, pcl::KdTreeFLANN<CloudData::POINT>::Ptr>(
            near_submap[i].index, kdtree_from_submap));
  }
  LoadSubMap(submap_id_global_, local_map_ptr_);
  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_from_submap(
      new pcl::KdTreeFLANN<CloudData::POINT>());
  kdtree_from_submap->setInputCloud(local_map_ptr_);

  registration_ptr_->SetInputTarget(local_map_ptr_, kdtree_from_submap);
  has_new_local_map_ = true;
  has_new_global_map_ = true;
  // submap_id_global_ = near_submap[0].index;
  return true;
}

void Filtering::SetFixPose(Eigen::Matrix4f& pose, float new_x, float new_y, float new_yaw) {
    Eigen::Matrix3f rotation;
    rotation << std::cos(new_yaw), -std::sin(new_yaw), 0,
               std::sin(new_yaw),  std::cos(new_yaw), 0,
               0,                 0,                 1;

    // 更新位姿矩阵
    pose.setIdentity();
    pose.block<3,3>(0,0) = rotation;
    pose(0, 3) = new_x;
    pose(1, 3) = new_y;
    pose(2, 3) = 0.0;
}

bool Filtering::SwitchLocalMap(const int num_local_map, Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &cloud_pose) {
  if(flag_start_hold_) {
      count_hold_++;
       SetFixPose(cloud_pose, x_hold_, y_hold_, theta_hold_);
       SetFixPose(predict_pose, x_hold_, y_hold_, theta_hold_);
       // SetFixPose(predict_pose, -4, 10, -1.57);
       // SetFixPose(cloud_pose, -4, 10, -1.57);
       if (count_hold_ > count_max_hold_) {
           flag_start_hold_ = false;
           count_hold_ = 0;
           ROS_ERROR(" SwitchLocalMap hold time finish ...... ");
      }
  }

  bool find_load_submap = false;
  if((num_local_map != submap_id_global_)) {
    lock_.lock();
    int32_t submap_id_global = submap_id_global_;
    ROS_INFO(" num_local_map: %d, submap_id_global: %d", num_local_map, submap_id_global);
    submap_id_global_ = num_local_map;
    if (map_around_submap_ptr_.find(submap_id_global_) != map_around_submap_ptr_.end()) {
      find_load_submap = true;
      local_map_ptr_->clear();
      local_map_ptr_ = map_around_submap_ptr_.at(submap_id_global_);
      kdtree_submap_ = map_around_submap_kdtree_ptr_.at(submap_id_global_);
      ROS_INFO(" SwitchLocalMap -> SetInputTarget ...... ");
      registration_ptr_->SetInputTarget(local_map_ptr_, kdtree_submap_);
      ROS_INFO(" SwitchLocalMap -> SetInputTarget finished ...... ");

      for (const auto& one_submap: submap_data_) {
        if(one_submap.index == submap_id_global_) {
            x_hold_ = one_submap.x;
            y_hold_ = one_submap.y;
            theta_hold_ = one_submap.z;
            SetFixPose(cloud_pose, x_hold_, y_hold_, theta_hold_);
            SetFixPose(predict_pose, x_hold_, y_hold_, theta_hold_);
            // SetFixPose(predict_pose, -4, 10, -1.57);
            // SetFixPose(cloud_pose, -4, 10, -1.57);
            count_hold_ = 0;
            flag_start_hold_ = true;
            ROS_INFO(" SwitchLocalMap -> SetFixPose: [%f, %f, %f]", one_submap.x, one_submap.y, one_submap.z);
            break;
        }
      }
    }
    lock_.unlock();
    has_new_local_map_ = true;
    has_new_global_map_ = true;

    if (!find_load_submap) {
      ROS_ERROR(" SwitchLocalMap , not_find_load_submap ...... ");
    }
    ROS_INFO(" SwitchLocalMap , need load near submaps ...... ");
  }
  return true;
}


} // namespace h_x