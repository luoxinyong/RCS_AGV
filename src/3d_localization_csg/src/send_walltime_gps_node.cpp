#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

ros::Publisher gps_pub;
nav_msgs::Odometry::ConstPtr gps_first_odom_ = nullptr;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  sensor_msgs::NavSatFix gps_data;
  double time = ros::Time::now().toSec();
  gps_data.header.stamp = ros::Time().fromSec(time);
  gps_data.header.frame_id = "navsat_link";
  gps_data.status = msg->status;
  gps_data.latitude = msg->latitude;
  gps_data.longitude = msg->longitude;
  gps_data.altitude = msg->altitude;
  gps_data.position_covariance = msg->position_covariance;
  gps_data.position_covariance_type = msg->position_covariance_type;
  gps_pub.publish(gps_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "republish_walltime_gps");
  cout << "republish_walltime_gps start" << endl;

  ros::NodeHandle nh_;
  gps_pub = nh_.advertise<sensor_msgs::NavSatFix>("/fix_wall_time", 10);
  ros::Subscriber gps_sub = nh_.subscribe("/fix", 10, gps_callback);
  ros::spin();
  return 0;
}
