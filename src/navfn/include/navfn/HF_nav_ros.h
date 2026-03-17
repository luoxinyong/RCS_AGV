#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>

namespace HF_nav_fn{
    class HF_nav_fn : public nav_core::BaseGlobalPlanner{
        public:
            HF_nav_fn();
            HF_nav_fn(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            HF_nav_fn(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
            ~HF_nav_fn();
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double cost);
        };
};
#endif