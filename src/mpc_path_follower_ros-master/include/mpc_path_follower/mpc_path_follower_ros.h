#ifndef _MPC_PATH_FOLLOWER_ROS_H
#define _MPC_PATH_FOLLOWER_ROS_H
#include <math.h>

#include <vector>

#include <geometry_msgs/Point32.h>
#include <mpc_path_follower/mpc_path_follower.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace mpc_path_follower
{
class MpcPathFollowerRos : public nav_core::BaseLocalPlanner
{
   public:
    MpcPathFollowerRos();
    ~MpcPathFollowerRos() = default;

    void initialize();
    bool ComputeVelocityCommands(const geometry_msgs::Twist& twist_current, const geometry_msgs::Pose& pose_current,
                                 geometry_msgs::Twist& cmd_vel);
    bool setPlan(std::vector<std::vector<geometry_msgs::Point32>>& orig_global_plan);

    bool isGoalReached();
    bool isInitialized()
    {
        return initialized_;
    }

   private:
    bool TransformGlobalPlan(const std::vector<geometry_msgs::Point32>& global_plan,
                             const geometry_msgs::Pose& pose_current,
                             std::vector<geometry_msgs::Point32>& transformed_plan);
    /**
     * @brief evaluate a polynominal
     * @param coefficients and input
     * @return output of the polynominal
     */
    int ClosestWaypoint(double x, double y, nav_msgs::Path global_path)
    {
    }
    /**
     * @brief evaluate a polynominal
     * @param coefficients and input
     * @return output of the polynominal
     */
    double polyeval(Eigen::VectorXd coeffs, double x);

    /**
     * @brief fit a polynominal
     * @param vector x, vector y and order
     * @return output coefficients
     */
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

    bool MpcComputeVelocityCommands(const std::vector<geometry_msgs::Point32>& path,
                                    const geometry_msgs::Twist& twist_current, const geometry_msgs::Pose& pose_current,
                                    geometry_msgs::Twist& cmd_vel);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    // tf2_ros::Buffer* tf_;
    ros::Publisher g_plan_pub_, l_plan_pub_;
    ros::Publisher pub_ref_path_odom_, pub_mpc_traj_, pub_ref_path_baselink_;

    bool initialized_ = false;
    bool debug_ = true;
    std::vector<geometry_msgs::Point32> global_plan_;

    float DT_ = 0.2;
    MPC_Path_Follower mpc_solver_;
    bool is_close_enough_;
    double dist_threshold_ = 3.0;
};
};  // namespace mpc_path_follower
#endif
