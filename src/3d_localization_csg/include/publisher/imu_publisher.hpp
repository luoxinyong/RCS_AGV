#ifndef SLAM_AD_PUBLISHER_IMU_PUBLISHER_HPP_
#define SLAM_AD_PUBLISHER_IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_data/imu_data.hpp"

namespace h_x {
class IMUPublisher {
  public:
    IMUPublisher(ros::NodeHandle& nh,
      const std::string& topic_name,
      const std::string& frame_id,
      const size_t& buff_size
    );
    IMUPublisher() = default;

    void Publish(const IMUData &imu_data, const double& time);
    void Publish(const IMUData &imu_data);

    bool HasSubscribers(void);

  private:
    void PublishData(const IMUData &imu_data, const ros::Time& time);

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::Imu imu_;
};
}

#endif