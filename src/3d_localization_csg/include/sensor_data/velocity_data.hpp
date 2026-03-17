#ifndef SLAM_AD_SENSOR_DATA_VELOCITY_DATA_HPP_
#define SLAM_AD_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace h_x {
    class VelocityData {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        public:
            struct LinearVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            };

            struct AngularVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            };

            double time = 0.0;
            LinearVelocity linear_velocity;
            AngularVelocity angular_velocity;
        
        public:
            static bool SyncData(std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>& UnsyncedData, std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>& SyncedData, const double& sync_time);
            void TransformCoordinate(Eigen::Matrix4f& transform_matrix);
            void NED2ENU(void);
};
}

#endif