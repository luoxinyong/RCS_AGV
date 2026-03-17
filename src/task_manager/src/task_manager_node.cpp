#include <stdio.h>

#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "task_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_manager");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    std::unique_ptr<task::TaskManager> ptr_task_manager_ = std::make_unique<task::TaskManager>(buffer);

    ros::Rate hz_pub(100);
    while (ros::ok()) {
        ptr_task_manager_->RunCycle();
        // std::cout<<"start"<<std::endl;
        ros::spinOnce();
        hz_pub.sleep();
    }
    return 0;
}
