#include "glog/logging.h"
#include "publisher/cloud_publisher.hpp"

namespace h_x {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               const std::string& topic_name,
                               const std::string& frame_id,
                               const size_t& buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input, const double& time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, const ros::Time& time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return (publisher_.getNumSubscribers() != 0);
}

}  // namespace h_x