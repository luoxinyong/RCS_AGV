#ifndef SLAM_AD_SENSOR_DATA_IMU_DATA_HPP_
#define SLAM_AD_SENSOR_DATA_IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace h_x {
class IMUData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  struct LinearAcceleration {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  class Orientation {
    public:
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double w = 0.0;

      void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
      }
  };

  double time = 0.0;
  LinearAcceleration linear_acceleration;
  AngularVelocity angular_velocity;
  Orientation orientation;

 public:
    // transform orientation to rotation matrix
    Eigen::Matrix3f GetOrientationMatrix() const;
    static bool SyncData(std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& UnsyncedData, std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& SyncedData, const double& sync_time);
};

}  // namespace h_x

#endif