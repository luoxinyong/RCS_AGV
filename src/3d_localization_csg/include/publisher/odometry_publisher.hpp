#ifndef SLAM_AD_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define SLAM_AD_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "sensor_data/velocity_data.hpp"

namespace h_x {
class OdometryPublisher {
  public:
    OdometryPublisher(ros::NodeHandle& nh, const std::string& topic_name,
                    const std::string& base_frame_id,
                    const std::string& child_frame_id, const int& buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, const double& time);
    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data, const double& time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel, const double& time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel);

    bool HasSubscribers();

  private:
    void PublishData(
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data, 
      const ros::Time& time
    );

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    VelocityData velocity_data_;
    nav_msgs::Odometry odometry_;
};
}  // namespace h_x
#endif