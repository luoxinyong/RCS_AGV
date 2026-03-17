#ifndef SLAM_AD_PUBLISHER_POINT_PUBLISHER_HPP_
#define SLAM_AD_PUBLISHER_POINT_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_data/imu_data.hpp"
#include "geometry_msgs/PointStamped.h"

namespace h_x {
class PointPublisher {
  public:
    PointPublisher(ros::NodeHandle& nh,
      const std::string& topic_name,
      const size_t& buff_size
    );
    PointPublisher() = default;

    void Publish(const geometry_msgs::PointStamped &point_data);
    bool HasSubscribers(void);

  private:

    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    geometry_msgs::PointStamped point_;
};
}

#endif