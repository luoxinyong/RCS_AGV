#ifndef SLAM_AD_SENSOR_DATA_GNSS_DATA_HPP_
#define SLAM_AD_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Geocentric/LocalCartesian.hpp"


namespace h_x {
class GNSSData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  double time = 0.0;
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  int status = 0;
  int service = 0;

 private:
  static GeographicLib::LocalCartesian geo_converter;
  static bool origin_position_inited;

 public:
  void InitOriginPosition(double& latitude, double& longitude, double& altitude);
  void UpdateXYZ();
  static bool SyncData(std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& UnsyncedData,
                       std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& SyncedData,
                       const double& sync_time);
};
}  // namespace h_x
#endif