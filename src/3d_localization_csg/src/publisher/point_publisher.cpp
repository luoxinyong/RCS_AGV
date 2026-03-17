#include "publisher/point_publisher.hpp"

#include "glog/logging.h"

namespace h_x {

PointPublisher::PointPublisher(ros::NodeHandle& nh, const std::string& topic_name,
                           const size_t& buff_size): nh_(nh) {
  publisher_ = nh_.advertise<geometry_msgs::PointStamped>(topic_name, buff_size);
}

void PointPublisher::Publish(const geometry_msgs::PointStamped& point_data) {

  // set data:
  // point_.x = point_data.x;
  // point_.y = point_data.y;
  // point_.z = point_data.z;

  publisher_.publish(point_data);
}

bool PointPublisher::HasSubscribers(void) {
  return (publisher_.getNumSubscribers() != 0);
}

}  // namespace h_x