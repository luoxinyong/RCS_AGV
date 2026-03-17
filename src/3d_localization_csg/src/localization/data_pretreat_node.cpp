#include <ros/ros.h>
#include "glog/logging.h"
#include "global_defination/global_defination.h"
#include "data_pretreat/data_pretreat_flow.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;
    
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<h_x::DataPretreatFlow> data_pretreat_flow_ptr = std::shared_ptr<h_x::DataPretreatFlow>(new h_x::DataPretreatFlow(nh, cloud_topic));
    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        data_pretreat_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}
