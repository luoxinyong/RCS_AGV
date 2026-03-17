#include <ros/ros.h>
#include "global_defination/global_defination.h"
#include "filtering/filtering_flow.hpp"
#include "glog/logging.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "filtering_node");
  ros::NodeHandle nh;

  std::shared_ptr<h_x::FilteringFlow> filtering_flow_ptr = std::shared_ptr<h_x::FilteringFlow>(new h_x::FilteringFlow(nh));

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    filtering_flow_ptr->Run();
    rate.sleep();
  }


  ROS_ERROR(" thread has been cancelled ");

  return 0;
}