#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
using namespace std;

ros::Publisher imu_pub;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  sensor_msgs::Imu imu_data;
  // imu_data.header.stamp = msg->header.stamp;
  double time = ros::Time::now().toSec();  // 把时间戳转化成浮点型格式
  imu_data.header.stamp = ros::Time().fromSec(time);  // 把浮点型变成时间戳
  imu_data.header.frame_id = "imu_link";
  imu_data.orientation = msg->orientation;
  imu_data.orientation_covariance = msg->orientation_covariance;
  imu_data.angular_velocity = msg->angular_velocity;
  imu_data.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_data.linear_acceleration = msg->linear_acceleration;
  imu_data.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  imu_pub.publish(imu_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "republish_walltime_imu");
  cout << " republish wall time imu start ...... " << endl;

  ros::NodeHandle nh_;
  imu_pub = nh_.advertise<sensor_msgs::Imu>("/imu_wall_time", 10);
  // ros::Subscriber imu_sub = nh_.subscribe("/oxts_ins/imu", 10, imu_callback);
  ros::Subscriber imu_sub = nh_.subscribe("/imu_raw", 10, imu_callback);

  ros::spin();
  return 0;
}
