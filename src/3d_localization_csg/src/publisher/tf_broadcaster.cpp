#include "publisher/tf_broadcaster.hpp"

namespace h_x {
TFBroadCaster::TFBroadCaster(const std::string& frame_id, const std::string& child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Matrix4f& pose, const double& time) {
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    transform_.stamp_ = ros::Time::now();
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
    broadcaster_.sendTransform(transform_);
}
}