#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;

ros::Publisher odom_pub;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  nav_msgs::Odometry odom_data;
  odom_data.header.stamp = ros::Time::now();
  odom_data.header.frame_id = "odom";
  odom_data.child_frame_id="base_footprint";

  odom_data.twist.twist.linear.x = 0;
  odom_data.twist.twist.linear.y = 0;
  odom_data.twist.twist.linear.z = 0;

  odom_pub.publish(odom_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "republish_walltime_odom");
  cout << "republish_walltime_odom start" << endl;

  ros::NodeHandle nh_;
  odom_pub = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Subscriber imu_sub = nh_.subscribe("/imu_raw", 10, imu_callback);
  ros::spin();
  return 0;
}
