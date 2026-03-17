#ifndef SLAM_AD_SENSOR_DATA_POSVEL_DATA_HPP_
#define SLAM_AD_SENSOR_DATA_POSVEL_DATA_HPP_

#include <string>
#include <Eigen/Dense>


namespace h_x {
    class PosVelData {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        public:
            double time = 0.0;
            Eigen::Vector3f pos = Eigen::Vector3f::Zero();
            Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    };
    
} // namespace h_x

#endif