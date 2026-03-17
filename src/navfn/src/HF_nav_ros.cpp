#include "HF_nav_ros.h"


namespace HF_nav_fn{

    HF_nav_fn::HF_nav_fn()
    {
    }
    HF_nav_fn::HF_nav_fn(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
    }

    void HF_nav_fn::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    }
    void HF_nav_fn::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){

    }
    bool HF_nav_fn::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        
    }
    bool HF_nav_fn::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double cost){

    }

};
