#include <ostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "glog/logging.h"
#include "global_defination/global_defination.h"
#include "filtering/filtering_flow.hpp"
#include "tools/file_manager.hpp"

namespace h_x {

FilteringFlow::FilteringFlow(ros::NodeHandle &nh) {
  std::string config_file_path = ros::package::getPath("loc_ad") + "/config/pretreat/pretreat.yaml";

  YAML::Node config_node  = YAML::LoadFile(config_file_path);
  std::string imu_topic   = config_node["imuTopic"].as<std::string>();
  std::string lidar_frame = config_node["lidarFrame"].as<std::string>();
  std::string base_frame  = config_node["baseFrame"].as<std::string>();
  count_update_correct_failed_ = 0;

  InitLocWithConfig();
  // caution, need to be verified.
  // subscriber:
  floor_raw_sub_ptr_ = std::make_shared<IntSubscriber>(nh, "num_floor", 10);
  // a. IMU raw measurement:
  imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic, 1000000);
  // b. undistorted Velodyne measurement:
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  // c. IMU synced measurement:
  imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
  // d. synced GNSS-odo measurement:
  pos_vel_sub_ptr_ = std::make_shared<PosVelSubscriber>(nh, "/synced_pos_vel", 100000);
  // e. lidar pose in map frame:
  gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
  // f. lidar to imu tf:
  base_to_lidar_ptr_ = std::make_shared<TFListener>(nh, lidar_frame, base_frame);

  // publisher:
  // a. global point cloud map:
  global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "map", 100);
  // b. local point cloud map:
  local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "map", 100);
  // c. current scan:
  current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "map", 100);
  // d. estimated lidar pose in map frame:
  laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "map", lidar_frame, 100);
  // e. fused pose in map frame:
  fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/fused_localization", "map", base_frame, 100);

  time_cost_exter_pub_ptr_ = std::make_shared<PointPublisher>(nh, "/time_cost_exter", 100);
  time_cost_while_pub_ptr_ = std::make_shared<PointPublisher>(nh, "/time_cost_while", 100);
  count_fail_pub_ptr_      = std::make_shared<PointPublisher>(nh, "/count_fail_send_fused_pose", 100);
  result_scan_match_pub_ptr_ = std::make_shared<PointPublisher>(nh, "/count_iter_scan_match", 100);
  time_wait_imu_lidar_ptr_   = std::make_shared<PointPublisher>(nh, "/time_wait_imu_lidar", 100);

  // f. tf:
  base_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("map", base_frame);

  filtering_ptr_ = std::shared_ptr<Filtering>(new Filtering());
  //读取局部地图数据
  filtering_ptr_->LoadSubmapData();
  filtering_ptr_->ShowSubmapData();
}

void FilteringFlow::InitLocWithConfig() {
  std::string config_filter = ros::package::getPath("loc_ad") + "/config/filtering/filtering.yaml";
  YAML::Node config_filter_node = YAML::LoadFile(config_filter);

  use_gnss_loc_init_ = config_filter_node["use_gnss_loc_init"].as<bool>();
  x_fix_init_pose_   = config_filter_node["fix_pose_loc_init"]["x_fix_init_pose"].as<double>();
  y_fix_init_pose_   = config_filter_node["fix_pose_loc_init"]["y_fix_init_pose"].as<double>();
  z_fix_init_pose_   = config_filter_node["fix_pose_loc_init"]["z_fix_init_pose"].as<double>();
  yaw_fix_init_pose_ = config_filter_node["fix_pose_loc_init"]["yaw_fix_init_pose"].as<double>();

  use_file_pose_init_ = config_filter_node["use_file_pose_init"].as<bool>();
  file_pose_init_path_ = config_filter_node["file_pose_init_path"].as<std::string>();
  use_sc_loc_init_ = config_filter_node["use_sc_loc_init"].as<bool>();
  scan_context_file_path_ = config_filter_node["map_path"].as<std::string>();
  scan_context_file_path_+="/ScanContext";
  std::string fusion_strategy_ = config_filter_node["fusion_strategy"].as<std::string>();
  //读取之前的点云描述子
  //sc_manager_ptr_ = std::shared_ptr<SCManager>(new SCManager());
  //std::vector<Eigen::MatrixXd> scanContext = loadSCDS(scan_context_file_path_ + "/SCDs/");
  //if (sc_manager_ptr_->setPolarContexts(scanContext)) {
  //  sc_manager_ptr_->setScanContextPath(scan_context_file_path_);
  //  ROS_INFO(" setPolarContexts success!");
  //}
  // 设置文件初始化默认路径
  if (file_pose_init_path_.empty()) {
    file_pose_init_path_ = ros::package::getPath("loc_ad") + "/saved_poses/latest_pose.txt";
  }
  
  // 确保目录存在
  // FileManager::CreateDirectory(FileManager::GetDirectory(file_pose_init_path_));
}

bool FilteringFlow::Run() {
  if (!InitCalibration()) {
    return false;
  }
  time_start_ = std::chrono::system_clock::now();
  static std::chrono::time_point<std::chrono::system_clock> time_start_wait_imulidar_ = std::chrono::system_clock::now();
  // if new global map is available, publish it:
  PublishGlobalMap();
  // if new local map is available, publish it:
  PublishLocalMap();
  time_pub_map_ = std::chrono::system_clock::now();
  elapsed_seconds_ = time_pub_map_ - time_start_;
  costtime_pub_map_ = elapsed_seconds_.count() * 1000;
  ReadData();
  time_read_data_ = std::chrono::system_clock::now();
  elapsed_seconds_ = time_read_data_ - time_pub_map_;
  costtime_read_data_ = elapsed_seconds_.count() * 1000;
  point_.header.stamp = ros::Time::now();
  point_.point.x = costtime_pub_map_;
  point_.point.y = costtime_read_data_;
  point_.point.z = 0;
  time_cost_exter_pub_ptr_->Publish(point_);
  // LOG(INFO) << " cost time:" << costtime_pub_map_ << " ," << costtime_read_data_ << std::endl;
  static int count_wait_imu_lidar = 0;
  while (HasData()) {
    count_wait_imu_lidar = 0;
    time_start_ = std::chrono::system_clock::now();
    time_start_wait_imulidar_ = std::chrono::system_clock::now();
    //第一次进入程序时的初始化操作，只执行一次，会将has_init_标志位设置为true，在filtering_ptr_中调用init函数
    if (!HasInited()) {
      if (ValidLidarData()) {
        if(!InitLocalization()) {
            // can be optimized: 1. wether pop out the current lidar cloud.
                              // 2 wether limit the numbers of scancontext.
            // std::cout<<"here!!!!"<<std::endl;

            break;
        }
      }
    } else {
      costtime_correct_ = 0;
      // std::cout<<"here!!!!2222222"<<std::endl;
      // handle timestamp chaos in an more elegant way
      if (HasLidarData() && ValidLidarData()) {
        if (HasIMUData()) {
          while (HasIMUData() && ValidIMUData() &&
                 current_imu_raw_data_.time < current_cloud_data_.time) {
            // next laser not arrive, use imu update pose
            UpdateLocalization();
          }

          // if (current_imu_raw_data_.time >= current_cloud_data_.time) {
          //   imu_raw_data_buff_.push_back(current_imu_raw_data_);
          // }
        }
        time_correct_start_ = std::chrono::system_clock::now();
        CorrectLocalization(floor_num_current_);
        // std::cout<<"here!!!!33333"<<std::endl;
        time_correct_end_ = std::chrono::system_clock::now();
        elapsed_seconds_ = time_correct_end_ - time_correct_start_;
        costtime_correct_ = elapsed_seconds_.count() * 1000;
      }
      time_s1_ = std::chrono::system_clock::now();
      elapsed_seconds_ = time_s1_ - time_start_;
      costtime_s1_ = elapsed_seconds_.count() * 1000;

      // no lidar data, exist imu data
      if (HasIMUData() && ValidIMUData()) {
        UpdateLocalization();
      }
      time_s2_ = std::chrono::system_clock::now();
      elapsed_seconds_ = time_s2_ - time_s1_;
      costtime_s2_ = elapsed_seconds_.count() * 1000;
    }
    time_while_ = std::chrono::system_clock::now();
    elapsed_seconds_ = time_while_ - time_start_;
    costtime_while_ = elapsed_seconds_.count() * 1000;
    data_time_while_.header.stamp = ros::Time::now();
    data_time_while_.point.x = costtime_correct_;
    data_time_while_.point.y = costtime_s1_;
    data_time_while_.point.z = costtime_s2_;
    time_cost_while_pub_ptr_->Publish(data_time_while_);
    // LOG(INFO) << " cost time:" << costtime_s1_ << " ," << costtime_s2_ << " ," << costtime_while_ <<std::endl;
  }
  count_wait_imu_lidar++;
  time_wait_imu_lidar_ = std::chrono::system_clock::now();
  elapsed_seconds_ = time_wait_imu_lidar_ - time_start_wait_imulidar_;
  costtime_wait_imu_lidar_ = elapsed_seconds_.count() * 1000;
  data_time_wait_imu_lidar_.header.stamp = ros::Time::now();
  data_time_wait_imu_lidar_.point.x = count_wait_imu_lidar;
  data_time_wait_imu_lidar_.point.y = costtime_wait_imu_lidar_;
  data_time_wait_imu_lidar_.point.z = 0;
  time_wait_imu_lidar_ptr_->Publish(data_time_wait_imu_lidar_);
  // LOG(INFO) << " S: count: " << count++ << std::endl;
  return true;
}

bool FilteringFlow::ReadData() {
    floor_raw_sub_ptr_->ParseData(floor_num_current_);
    // pipe raw IMU measurements into buffer:
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    // 过滤掉IMU数据中时间戳小于当前时间的旧数据
    while (HasInited() && HasIMUData() && 
           imu_raw_data_buff_.front().time < filtering_ptr_->GetTime()) {
        imu_raw_data_buff_.pop_front();
  }

  // pipe synced lidar-GNSS-IMU measurements into buffer:

  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
  pos_vel_sub_ptr_->ParseData(pos_vel_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);

  //  LOG(INFO) << " Read laser->imu->vel->gnss size : " << cloud_data_buff_.size() << "," << imu_synced_data_buff_.size() << ","
  //  << pos_vel_data_buff_.size() << "," << gnss_data_buff_.size() << ","   << std::endl;

  return true;
}

bool FilteringFlow::HasInited(void) { 
  return filtering_ptr_->HasInited(); 
}

bool FilteringFlow::HasData() {
  if (!HasInited()) {
      //  LOG(INFO) << " HasInited is false: "  << std::endl;
    if (!HasLidarData()) {
        // LOG(INFO) << " HasLidarData is false: "  << std::endl;
      return false;
    }
  } else {
    if (!HasIMUData() && !HasLidarData()) {
      // LOG(INFO) << " HasIMUData: " << HasIMUData() << ", HasLidarData: " << HasLidarData() << std::endl;
      return false;
    }
  }

  return true;
}
//拿出当前IMU原始数据的第一帧保存到current_imu_raw_data_里面，然后从IMU原始数据缓冲区中移除这一帧数据
bool FilteringFlow::ValidIMUData() {
  current_imu_raw_data_ = imu_raw_data_buff_.front();
  imu_raw_data_buff_.pop_front();
  return true;
}

// 检查 LiDAR 数据是否与 IMU 及位姿/速度数据同步，时间差小于0.05s
bool FilteringFlow::ValidLidarData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_synced_data_ = imu_synced_data_buff_.front();
  current_pos_vel_data_ = pos_vel_data_buff_.front();

  double diff_imu_time = current_cloud_data_.time - current_imu_synced_data_.time;
  double diff_pos_vel_time = current_cloud_data_.time - current_pos_vel_data_.time;

  if (diff_imu_time < -0.05 || diff_pos_vel_time < -0.05) {
    cloud_data_buff_.pop_front();
    ROS_INFO(" ValidLidarData: laser too front, diff_imu_time: %f, diff_pos_vel_time: %f", diff_imu_time, diff_pos_vel_time);
    return false;
  }

  if (diff_imu_time > 0.05) {
    imu_synced_data_buff_.pop_front();
    ROS_INFO(" ValidLidarData: imu too front, diff_imu_time: %f", diff_imu_time);
    return false;
  }

  if (diff_pos_vel_time > 0.05) {
    pos_vel_data_buff_.pop_front();
    ROS_INFO(" ValidLidarData: vel too front, diff_pos_vel_time: %f", diff_pos_vel_time);
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  pos_vel_data_buff_.pop_front();

  return true;
}

bool FilteringFlow::InitCalibration() {
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (base_to_lidar_ptr_->LookupData(base_to_laser_)) {
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool FilteringFlow::InitLocalization(void) {
  // ego vehicle velocity in body frame:
  Eigen::Vector3f init_vel = current_pos_vel_data_.vel;
  Eigen::Matrix4f init_pose = gnss_data_buff_.front().pose;

  bool file_pose_used = false;
  bool init_success = false;
  pose_init_time_ = std::chrono::system_clock::now();
  if (use_file_pose_init_) {
    std::ifstream file(file_pose_init_path_);
    if (file.is_open()) {
      std::string line;
      if (std::getline(file, line)) {
        std::istringstream iss(line);
        double timestamp, x, y, z, qx, qy, qz, qw;
        
        if (iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
          // 构造位姿矩阵
          Eigen::Quaternionf q(qw, qx, qy, qz);
          q.normalize();
          
          init_pose.block<3,3>(0,0) = q.toRotationMatrix();
          init_pose(0,3) = x;
          init_pose(1,3) = y;
          init_pose(2,3) = z;
          
          ROS_INFO("Loaded init pose from file: [%.3f, %.3f, %.3f]", x, y, z);
          file_pose_used = true;
          
          // 尝试使用文件位姿初始化
          if (filtering_ptr_->Init( current_cloud_data_, init_pose, init_vel,
                          current_imu_synced_data_)) {
            ROS_INFO(" Use File InitLocalization successed ...... ");
            init_success = true;
          }else{
            ROS_INFO(" Use File InitLocalization failed ...... ");
            init_success = false;
          }
      
        }
      }
      file.close();
    }
  }
  pose_init_end_time = std::chrono::system_clock::now();
  time_pose_init_cost_ = pose_init_end_time - pose_init_time_;
  std::cout<<"file pose init cost time: "<<time_pose_init_cost_.count()*1000<<"ms"<<std::endl;

  pose_init_time_ = std::chrono::system_clock::now();
  if(!init_success){
    ROS_INFO(" Use File InitLocalization failed, try gnss init ...... ");
      if(!use_gnss_loc_init_) {
        try {
          Eigen::Vector3f t;
          t.x() = x_fix_init_pose_;
          t.y() = y_fix_init_pose_;
          t.z() = z_fix_init_pose_;
          init_pose.block<3, 1>(0, 3) = t;
          Eigen::AngleAxisf yaw_pose_orientation(yaw_fix_init_pose_, Eigen::Vector3f::UnitZ());
          init_pose.block<3, 3>(0, 0) = yaw_pose_orientation.toRotationMatrix();
        } catch(const std::exception& e) {
          LOG(ERROR) << " Load yaml init pose error: " << e.what() << std::endl;
        }
        LOG(INFO) << " yaml init pose: " << init_pose << std::endl;
      }
      if(use_sc_loc_init_) {
        gnss_data_buff_.pop_front();
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_SC_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*current_cloud_data_.cloud_ptr, *current_SC_cloud);

        if(!setScanContextPose(*current_SC_cloud, init_pose)) {
          return false;
        }
      }
      if (filtering_ptr_->Init( current_cloud_data_, init_pose, init_vel,
                          current_imu_synced_data_)) {
          ROS_INFO(" Use Gnss InitLocalization successed ...... ");
            init_success = true;
          }else{
            ROS_INFO(" Use Gnss InitLocalization failed ...... ");
            init_success = false;
          }
  }
    pose_init_end_time = std::chrono::system_clock::now();
    time_pose_init_cost_ = pose_init_end_time - pose_init_time_;
    std::cout<<"gnss pose init cost time: "<<time_pose_init_cost_.count()*1000<<"ms"<<std::endl;
      
    
    if (init_success) {
        ROS_INFO("InitLocalization succeeded");
        return true;
      } else {
        ROS_ERROR("InitLocalization failed");
        return false;
      }
  // bool file_pose_used = false;
  // bool init_success = false;

  // if (use_file_pose_init_) {
  //   std::ifstream file(saved_pose_file_path_);
  //   if (file.is_open()) {
  //     std::string line;
  //     if (std::getline(file, line)) {
  //       std::istringstream iss(line);
  //       double timestamp, x, y, z, qx, qy, qz, qw;
        
  //       if (iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
  //         // 构造位姿矩阵
  //         Eigen::Quaternionf q(qw, qx, qy, qz);
  //         q.normalize();
          
  //         init_pose.block<3,3>(0,0) = q.toRotationMatrix();
  //         init_pose(0,3) = x;
  //         init_pose(1,3) = y;
  //         init_pose(2,3) = z;
          
  //         ROS_INFO("Loaded init pose from file: [%.3f, %.3f, %.3f]", x, y, z);
  //         file_pose_used = true;
          
  //         // 尝试使用文件位姿初始化
  //         init_success = filtering_ptr_->Init(current_cloud_data_, init_pose, 
  //                                             init_vel, current_imu_synced_data_);
          
  //         // 检查匹配质量
  //         if (init_success) {
  //           float fitness_score = filtering_ptr_->GetInitFitnessScore();
  //           ROS_INFO("File pose init fitness score: %.3f", fitness_score);
            
  //           if (fitness_score > file_pose_fitness_threshold_) {
  //             ROS_WARN("File pose init failed (score %.3f > threshold %.3f). "
  //                      "Trying fallback init.", 
  //                      fitness_score, file_pose_fitness_threshold_);
  //             init_success = false;
  //           }
  //         }
  //       }
  //     }
  //     file.close();
  //   }
  // }

  // if(!use_gnss_loc_init_) {
  //   try {
  //     Eigen::Vector3f t;
  //     t.x() = x_fix_init_pose_;
  //     t.y() = y_fix_init_pose_;
  //     t.z() = z_fix_init_pose_;
  //     init_pose.block<3, 1>(0, 3) = t;
  //     Eigen::AngleAxisf yaw_pose_orientation(yaw_fix_init_pose_, Eigen::Vector3f::UnitZ());
  //     init_pose.block<3, 3>(0, 0) = yaw_pose_orientation.toRotationMatrix();
  //   } catch(const std::exception& e) {
  //     LOG(ERROR) << " Load yaml init pose error: " << e.what() << std::endl;
  //   }
  //   LOG(INFO) << " yaml init pose: " << init_pose << std::endl;
  // }

  // if(use_sc_loc_init_) {
  //   gnss_data_buff_.pop_front();
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr current_SC_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  //   pcl::copyPointCloud(*current_cloud_data_.cloud_ptr, *current_SC_cloud);

  //   if(!setScanContextPose(*current_SC_cloud, init_pose)) {
  //     return false;
  //   }
  // }

  // // use gnss init pose
  // if (filtering_ptr_->Init( current_cloud_data_, init_pose, init_vel,
  //                         current_imu_synced_data_)) {
  //   ROS_INFO(" InitLocalization successed ...... ");
  // } else {
  //   ROS_INFO(" InitLocalization failed ...... ");
  //   return false;
  // }

  // return true;
}

bool FilteringFlow::UpdateLocalization() {
  if (filtering_ptr_->Update(current_imu_raw_data_)) {
    PublishFusionOdom();
    count_update_correct_failed_ = 0;
    return true;
  }
  count_update_correct_failed_ ++;
  count_failed_pose_.header.stamp = ros::Time::now();
  count_failed_pose_.point.x = count_update_correct_failed_;
  count_fail_pub_ptr_->Publish(count_failed_pose_);
  return false;
}

bool FilteringFlow::CorrectLocalization(const int num_local_map) {
  double startTime = ros::Time::now().toSec();
  bool is_fusion_succeeded =
      filtering_ptr_->Correct(num_local_map, current_imu_synced_data_, current_cloud_data_,
                              current_pos_vel_data_, laser_pose_);
  int count_scan_match = 0;
  float score_fitness = 0;
  float costtime_scan_match = 0;
  filtering_ptr_->GetScanMatchResult(count_scan_match, score_fitness, costtime_scan_match);
  double endTime = ros::Time::now().toSec();
  // std::cout << " CorrectLocalization cost time: " << std::setprecision(6) << (endTime-startTime) << std::endl;
  PublishLidarOdom();
  //
  result_scan_match_.header.stamp = ros::Time::now();
  result_scan_match_.point.x = static_cast<float>(count_scan_match);
  result_scan_match_.point.y = static_cast<float>(score_fitness*10.0);
  result_scan_match_.point.z = static_cast<float>(costtime_scan_match);
  result_scan_match_pub_ptr_->Publish(result_scan_match_);

  if (is_fusion_succeeded) {
    PublishFusionOdom();

    // add to odometry output for evo evaluation:
    UpdateOdometry(current_cloud_data_.time);
    count_update_correct_failed_ = 0;
    return true;
  }
  count_update_correct_failed_ ++;
  count_failed_pose_.header.stamp = ros::Time::now();
  count_failed_pose_.point.x = count_update_correct_failed_;
  count_fail_pub_ptr_->Publish(count_failed_pose_);
  return false;
}

bool FilteringFlow::PublishGlobalMap() {
  if (filtering_ptr_->HasNewGlobalMap() &&
      global_map_pub_ptr_->HasSubscribers()) {
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    filtering_ptr_->GetGlobalMap(global_map_ptr);
    global_map_pub_ptr_->Publish(global_map_ptr);
    // std::cout << " global map pub: " << std::endl;
    return true;
  }
  return false;
}

bool FilteringFlow::PublishLocalMap() {
  if (filtering_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
    local_map_pub_ptr_->Publish(filtering_ptr_->GetLocalMap());
    std::cout << " local map pub: " << std::endl;
    return true;
  }
  return false;
}

bool FilteringFlow::PublishLidarOdom() {
  // a. publish lidar odometry
  laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.time);
  // b. publish current scan:
  current_scan_pub_ptr_->Publish(filtering_ptr_->GetCurrentScan());
  return true;
}

bool FilteringFlow::PublishFusionOdom() {
  // get odometry from Kalman filter:
  filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);
  fused_pose_ *= base_to_laser_; // transform lidar pose to base

  double dx = fused_pose_(0,3) - xx_;
  double dy = fused_pose_(1,3) - yy_;
  double dz = fused_pose_(2,3) - zz_;
  if(sqrt( pow(dx, 2.0) + pow(dy, 2.0) + pow(dz, 2.0)) > 0.1) {
    count_jump_++;
    ROS_ERROR(" loc pose jumped, count_jump: %d, delta: [%f, %f, %f]", count_jump_, dx, dy, dz);
  }

  // set the pose
  xx_ = fused_pose_(0,3);
  yy_ = fused_pose_(1,3);
  zz_ = fused_pose_(2,3);

  // a. publish tf:
  base_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.time);
  // b. publish fusion odometry:
  fused_odom_pub_ptr_->Publish(fused_pose_, fused_vel_,
                               current_imu_raw_data_.time);

  if (use_file_pose_init_) {

    std::string tmp_path = file_pose_init_path_ + ".tmp";
    const double timestamp = current_imu_raw_data_.time;
    Eigen::Quaternionf q(fused_pose_.block<3,3>(0,0));

    {
      std::ofstream pose_file(tmp_path, std::ios::trunc);
      if (pose_file.is_open()) {
        pose_file << std::fixed << std::setprecision(6)
                  << timestamp << " "
                  << fused_pose_(0,3) << " "
                  << fused_pose_(1,3) << " "
                  << fused_pose_(2,3) << " "
                  << q.x() << " "
                  << q.y() << " "
                  << q.z() << " "
                  << q.w() << "\n";
        pose_file.flush();

        // 2. 确保落盘
        int fd = ::open(tmp_path.c_str(), O_RDWR);
        if (fd != -1) {
          ::fsync(fd);
          ::close(fd);
        }
      } else {
        ROS_ERROR("Failed to save pose to tmp file: %s", tmp_path.c_str());
      }
    }

    // 3. 原子替换：保证不会出现空文件
    if (::rename(tmp_path.c_str(), file_pose_init_path_.c_str()) != 0) {
      ROS_ERROR("Failed to rename tmp file to %s", file_pose_init_path_.c_str());
    }
  }
  return true;
}

bool FilteringFlow::UpdateOdometry(const double &time) {
  trajectory.time_.push_back(time);
  trajectory.fused_.push_back(fused_pose_);
  trajectory.lidar_.push_back(laser_pose_);
  ++trajectory.N;
  return true;
}

/**
 * @brief  save pose in KITTI format for evo evaluation
 * @param  pose, input pose
 * @param  ofs, output file stream
 * @return true if success otherwise false
 */
bool FilteringFlow::SavePose(const Eigen::Matrix4f &pose,
                                  std::ofstream &ofs) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }

  return true;
}

bool FilteringFlow::setScanContextPose(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Matrix4f &init_pose)
{
	Eigen::Vector3d GnssInitPose; 
  GnssInitPose << (double)init_pose(0,3), (double)init_pose(1,3), (double)init_pose(2,3);	
	std::cout << "The current pose of the GPS is: x: " << init_pose(0,3) << "y: " << init_pose(1,3) <<"z: " << init_pose(2,3)<<std::endl;
	sc_manager_ptr_->makeAndSaveScancontextAndKeys(scan);
	auto detectResult = sc_manager_ptr_->detectLoopClosureID(GnssInitPose);
	int loopKeyPre = detectResult.first;
	float yawDiffRad = detectResult.second; 
	if( loopKeyPre == -1 /* No loop found */)	{
	    sc_manager_ptr_->dropScancontextAndKeys();
	    cout<<"No loop found!"<<endl;
	    return false;
	}	else {
	    cout<<"The loopKeyPre: "<<loopKeyPre<<endl<<"The yawDiffRad: "<<yawDiffRad<<endl;
	}

	loadSCPose(scan_context_file_path_ + "/optimized_poses.txt", copy_CloudKeyPoses);
	std::cout << "copy_CloudKeyPoses: \n" << copy_CloudKeyPoses.size() << std::endl;
	if (loopKeyPre < 0 && loopKeyPre >= copy_CloudKeyPoses.size()) {
	    return false;
	}
	// Eigen::Matrix3d KeyPrePose = copy_CloudKeyPoses.at(loopKeyPre).leftCols(3);
	// Eigen::Vector3d T_XYZ = copy_CloudKeyPoses.at(loopKeyPre).col(4);
	// Eigen::Vector3d R_XYZ = KeyPrePose.eulerAngles(0,1,2);

	auto KeyPrePose_temp = copy_CloudKeyPoses.at(loopKeyPre);
	cout<<"KeyPrePose_temp: \n"<<KeyPrePose_temp<<endl;
	Eigen::Matrix3d KeyPrePose = KeyPrePose_temp.leftCols(3);
	cout<<"KeyPrePose: \n"<<KeyPrePose<<endl;
	Eigen::Vector3d T_XYZ = KeyPrePose_temp.col(3);
	cout<<"T_XYZ: \n"<<T_XYZ<<endl;
	Eigen::Vector3d R_XYZ = KeyPrePose.eulerAngles(0,1,2);
	cout<<"R_XYZ: \n"<<R_XYZ<<endl;
      	
	Eigen::Affine3f Sc_pose = pcl::getTransformation(T_XYZ(0), T_XYZ(1), T_XYZ(2), R_XYZ(0),R_XYZ(1),R_XYZ(2)+yawDiffRad); 
	
	init_pose = Sc_pose.matrix();

	std::cout << "SC loop found! " << std::endl;
	return true; 
}

Eigen::MatrixXd FilteringFlow::loadSCD(const std::string file_path)
{
    // read line by line, store each line into a cpp vector!
    // map vector object to eigen: Map(cpp_array) to Matrix
    vector<double> matrixEntries;
    // in this object we store the data from the matrix
    ifstream matrixDataFile(file_path);
    string matrixRowString;
    string matrixEntry;
    int matrixRowNumber = 0;

    while (getline(matrixDataFile, matrixRowString)) {
        stringstream matrixRowStringStream(matrixRowString);
        while(getline(matrixRowStringStream, matrixEntry, ' ')) {
            //convert the string to double and fill in the array cpp vector
            matrixEntries.push_back(stod(matrixEntry));
        }
        // copy(matrixEntries.begin(), matrixEntries.end(), ostream_iterator<double>(cout, " "));
        matrixRowNumber++;
    }
    // std::cout<<"matrixRowNumber: "<<matrixRowNumber<<std::endl;
    //here the matrixEntries.data() is the pointer to the first memory location
    //at which entries of the vector matrixEntries are stored;
    return Eigen::Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(), matrixRowNumber, matrixEntries.size()/matrixRowNumber);
}

std::vector<Eigen::MatrixXd> FilteringFlow::loadSCDS(std::string dir_path) 
{
    DIR* dir;
    struct dirent* ent;
    vector<string> names;
    vector<Eigen::MatrixXd> polarContexts;
    if ((dir = opendir(dir_path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
                names.push_back(ent->d_name);
        }
        closedir(dir);
    }
    sort(names.begin(), names.end());
    // copy(names.begin(), names.end(), ostream_iterator<string>(cout, " "));
    std::cout << "SC file number: " << names.size() << std::endl;
    for (auto& file:names) {
        // loadSCD(dir_path + file, polarContext);
        polarContexts.push_back(loadSCD(dir_path + file));
    }
    cout <<" polarContexts: " << polarContexts.size() << endl;
    cout <<" The last polarContexts: " << polarContexts.back().bottomRows(1) << endl;
    // sc_manager_ptr_->setPolarContexts(polarContexts);
    return polarContexts;
}

void FilteringFlow::loadSCPose(const std::string& file_path, std::vector<Eigen::MatrixXd>&CloudKeyPoses)
{
    vector<double> matrixEntries;
    ifstream matrixDataFile(file_path);
    string matrixRowString;
    string matrixEntry;
    while (getline(matrixDataFile, matrixRowString)) {
        stringstream matrixRowStringStream(matrixRowString);
        while(getline(matrixRowStringStream, matrixEntry, ' ')) {
            //convert the string to double and fill in the array cpp vector
            matrixEntries.push_back(stod(matrixEntry));
            // CloudKeyPoses.push_back(Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(), 4, 4));            
        }
        // copy(matrixEntries.begin(), matrixEntries.end(), ostream_iterator<double>(cout, " "));
        // std::cout<<endl;
        CloudKeyPoses.push_back(Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(), 3, 4));
        matrixEntries.clear();
    }
    std::cout << "CloudKeyPoses: " << CloudKeyPoses.size() << std::endl;
    std::cout << "CloudKeyPoses last element(MatrixXd): " << CloudKeyPoses.back() << std::endl;
}

}
