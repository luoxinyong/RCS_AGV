#ifndef SLAM_AD_PUBLISHER_POSVEL_PUBLISHER_HPP_
#define SLAM_AD_PUBLISHER_POSVEL_PUBLISHER_HPP_

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "sensor_data/pos_vel_data.hpp"
#include "loc_ad/PosVel.h"

namespace h_x {
class PosVelPublisher {
  public:
    PosVelPublisher(
      ros::NodeHandle& nh, 
      const std::string& topic_name, 
      const std::string& base_frame_id,
      const std::string& child_frame_id,
      const int& buff_size
    );
    PosVelPublisher() = default;

    void Publish(const PosVelData &pos_vel_data, const double &time);
    void Publish(const PosVelData &pos_vel_data);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    loc_ad::PosVel pos_vel_msg_;
    void PublishData(const PosVelData &pos_vel_data, const ros::Time& time);
};
}






#endif