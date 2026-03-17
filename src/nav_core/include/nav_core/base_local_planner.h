#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

namespace nav_core
{
/**
 * @class 局部路径规划器基类，后续子类需要继承该类，并通过插件化注册该类
 * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the
 * navigation stack must adhere to this interface.
 */
class BaseLocalPlanner
{
   public:
    /**
     * @brief  根据给定的机器人当前位置、姿态、速度计算并返回机器人的速度命令，纯虚函数，需要子类继承并实现方可调用
     * @param twist_current 当前机器人的速度
     * @param pose_current 当前机器人的位置和姿态
     * @param cmd_vel 计算得到的速度命令
     * @return True if a valid velocity command was found, false otherwise
     */
    virtual bool ComputeVelocityCommands(const geometry_msgs::Twist& twist_current,
                                         const geometry_msgs::Pose& pose_current, geometry_msgs::Twist& cmd_vel) = 0;

    /**
     * @brief  检查局部规划器是否已到达目标点（局部的目标点）
     * @return True if achieved, false otherwise
     */
    virtual bool isGoalReached() = 0;

    /**
     * @brief  接收来自全局路径规划器规划出来的全局路径
     * @param plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    virtual bool setPlan(std::vector<std::vector<geometry_msgs::Point32>>& orig_global_plan) = 0;

    /**
     * @brief  局部路径规划器初始化函数
     * @param name The name to give this instance of the local planner
     * @param tf A pointer to a transform listener
     * @param costmap_ros The cost map to use for assigning costs to local plans
     */
    virtual void initialize() = 0;
    /**
     * @brief  Virtual destructor for the interface
     */
    virtual ~BaseLocalPlanner()
    {
    }

   protected:
    BaseLocalPlanner()
    {
    }
};
};  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
