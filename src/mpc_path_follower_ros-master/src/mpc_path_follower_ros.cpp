#include <mpc_path_follower/mpc_path_follower_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(mpc_path_follower::MpcPathFollowerRos, nav_core::BaseLocalPlanner)

namespace mpc_path_follower
{
MpcPathFollowerRos::MpcPathFollowerRos() : initialized_(false), is_close_enough_(false)
{
}

void MpcPathFollowerRos::initialize()
{
    if (!isInitialized()) {
        ros::NodeHandle nh;
        pub_ref_path_odom_ = nh.advertise<nav_msgs::Path>("/mpc_reference_path_odom", 1);
        pub_ref_path_baselink_ = nh.advertise<nav_msgs::Path>("/mpc_reference_path_baselink", 1);
        pub_mpc_traj_ = nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);  // MPC traj output

        initialized_ = true;
    } else {
        ROS_WARN(" This planner has already been initialized, doing nothing.");
    }
}

bool MpcPathFollowerRos::TransformGlobalPlan(const std::vector<geometry_msgs::Point32> &global_plan,
                                             const geometry_msgs::Pose &pose_current,
                                             std::vector<geometry_msgs::Point32> &transformed_plan)
{
    transformed_plan.clear();

    if (global_plan.empty()) {
        ROS_ERROR("Received plan with zero length");
        return false;
    }

    try {
        unsigned int i = 0;
        double sq_dist_threshold = dist_threshold_ * dist_threshold_;
        double sq_dist = 0;

        // we need to loop to a point on the plan that is within a certain distance of the robot
        while (i < (unsigned int)global_plan.size()) {
            double x_diff = pose_current.position.x - global_plan[i].x;
            double y_diff = pose_current.position.y - global_plan[i].y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (sq_dist <= sq_dist_threshold) {
                break;
            }
            ++i;
        }

        geometry_msgs::Point32 newer_pose;

        // now we'll transform until points are outside of our distance threshold
        while (i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
            const geometry_msgs::Point32 &pose = global_plan[i];

            newer_pose = pose;
            // tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            double x_diff = pose_current.position.x - global_plan[i].x;
            double y_diff = pose_current.position.y - global_plan[i].y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;

            ++i;
        }
    } catch (tf2::LookupException &ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    } catch (tf2::ConnectivityException &ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    } catch (tf2::ExtrapolationException &ex) {
        ROS_ERROR("Extrapolation Error 1: %s\n", ex.what());
        return false;
    }

    return true;
}

bool MpcPathFollowerRos::ComputeVelocityCommands(const geometry_msgs::Twist &twist_current,
                                                 const geometry_msgs::Pose &pose_current, geometry_msgs::Twist &cmd_vel)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    std::vector<geometry_msgs::Point32> transformed_plan;
    transformed_plan.clear();
    if (!TransformGlobalPlan(global_plan_, pose_current, transformed_plan)) {
        return false;
    }
    if (transformed_plan.empty()) {
        ROS_ERROR("Received an empty transformed plan.");
        return false;
    }
    ROS_INFO("Received a transformed plan with %zu points.", transformed_plan.size());

    if (isGoalReached()) {
        return true;
    } else {
        bool isOk = MpcComputeVelocityCommands(transformed_plan, twist_current, pose_current, cmd_vel);
        if (!isOk) {
            ROS_ERROR(" mpc planner failed to produce path.");
        }
        return isOk;
    }
}

bool MpcPathFollowerRos::MpcComputeVelocityCommands(const std::vector<geometry_msgs::Point32> &path,
                                                    const geometry_msgs::Twist &twist_current,
                                                    const geometry_msgs::Pose &pose_current,
                                                    geometry_msgs::Twist &cmd_vel)
{
    Eigen::Vector3f vel(twist_current.linear.x, twist_current.linear.y, twist_current.angular.z);

    // Display the MPC reference trajectory in odom coordinate
    nav_msgs::Path _mpc_ref_traj;
    _mpc_ref_traj.header.frame_id = "map";
    _mpc_ref_traj.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped tempPose;
    tempPose.header = _mpc_ref_traj.header;
    for (int i = 0; i < path.size(); ++i) {
        tempPose.pose.position.x = path[i].x;
        tempPose.pose.position.y = path[i].y;
        _mpc_ref_traj.poses.push_back(tempPose);
    }
    pub_ref_path_odom_.publish(_mpc_ref_traj);

    // nav_msgs::Odometry odom;
    // odom_helper_.getOdom(odom);
    double px = pose_current.position.x;  // pose: odom frame
    double py = pose_current.position.y;

    double psi = tf2::getYaw(pose_current.orientation);
    // Waypoints related parameters
    double cospsi = cos(psi);
    double sinpsi = sin(psi);
    // Convert to the vehicle coordinate system
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    waypoints_x.clear();
    waypoints_y.clear();

    // Display the MPC reference trajectory in odom coordinate
    nav_msgs::Path _vehicle_ref_traj;
    _vehicle_ref_traj.header.frame_id = "chassis_link";  // points in car coordinate
    _vehicle_ref_traj.header.stamp = ros::Time::now();
    tempPose.header = _vehicle_ref_traj.header;
    for (int i = 0; i < path.size(); i++) {
        double dx = path.at(i).x - px;
        double dy = path.at(i).y - py;
        waypoints_x.push_back(dx * cospsi + dy * sinpsi);
        waypoints_y.push_back(dy * cospsi - dx * sinpsi);
        tempPose.pose.position.x = dx * cospsi + dy * sinpsi;
        tempPose.pose.position.y = dy * cospsi - dx * sinpsi;
        _vehicle_ref_traj.poses.push_back(tempPose);
    }
    pub_ref_path_baselink_.publish(_vehicle_ref_traj);
    int size_of_path = waypoints_x.size();
    if (size_of_path <= 6) {
        is_close_enough_ = true;
        return true;
    }
    is_close_enough_ = false;
    double *ptrx = &waypoints_x[0];
    double *ptry = &waypoints_y[0];
    Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, size_of_path);
    Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, size_of_path);
    // calculate cte and epsi
    auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
    /* The cross track error is calculated by evaluating at polynomial at x, f(x)
    and subtracting y.
    double cte = polyeval(coeffs, x) - y;
    Due to the sign starting at 0, the orientation error is -f'(x).
    derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
    double epsi = psi - atan(coeffs[1]);*/
    double cte = polyeval(coeffs, 0);
    double epsi = atan(coeffs[1]);
    if (debug_) {
        std::cout << "psi is" << std::endl;
        std::cout << "path size is" << path.size() << std::endl;
        std::cout << "waypoints x size is" << waypoints_x.size() << std::endl;
        std::cout << "coeffs is " << coeffs << std::endl;
        std::cout << "cte is" << cte << std::endl;
        std::cout << "epsi is" << epsi << std::endl;
    }
    Eigen::VectorXd state(6);
    state << 0, 0, 0, vel[0], cte, epsi;
    std::vector<double> vars;
    vars.clear();
    vars = mpc_solver_.solve(state, coeffs);
    if (vars.size() < 2) {
        return false;
    }
    std::vector<double> mpc_x_vals;
    std::vector<double> mpc_y_vals;
    mpc_x_vals.clear();
    mpc_y_vals.clear();
    for (int i = 2; i < vars.size(); i++) {
        if (i % 2 == 0) {
            mpc_x_vals.push_back(vars[i]);
        } else {
            mpc_y_vals.push_back(vars[i]);
        }
    }

    // Display the MPC predicted trajectory
    nav_msgs::Path _mpc_predi_traj;
    _mpc_predi_traj.header.frame_id = "chassis_link";  // points in car coordinate
    _mpc_predi_traj.header.stamp = ros::Time::now();
    tempPose.header = _mpc_predi_traj.header;
    for (int i = 2; i < mpc_x_vals.size() - 3; i++) {
        tempPose.pose.position.x = mpc_x_vals[i];
        tempPose.pose.position.y = mpc_y_vals[i];
        tempPose.pose.orientation.w = 1.0;
        _mpc_predi_traj.poses.push_back(tempPose);
    }
    pub_mpc_traj_.publish(_mpc_predi_traj);

    double steer_value = 0.0, throttle_value = 0.0;
    steer_value = vars[0];
    throttle_value = vars[1];
    ROS_INFO("Steer value and throttle value is, %lf , %lf", steer_value, throttle_value);
    cmd_vel.linear.x = vel[0] + vars[1] * DT_;
    double radius = 0.0;
    if (fabs(tan(steer_value)) <= 1e-2) {
        radius = 1e5;
    } else {
        radius = 0.5 / tan(steer_value);
    }
    cmd_vel.angular.z = std::max(-1.0, std::min(1.0, (cmd_vel.linear.x / radius)));
    cmd_vel.linear.x = std::min(0.2, cmd_vel.linear.x);
    ROS_INFO("v value and z value is, %lf , %lf", cmd_vel.linear.x, cmd_vel.angular.z);
    return true;
}

bool MpcPathFollowerRos::setPlan(std::vector<std::vector<geometry_msgs::Point32>> &orig_global_plan)
{
    global_plan_.clear();
    for (const auto &sub_plan : orig_global_plan) {
        global_plan_.insert(global_plan_.end(), sub_plan.begin(), sub_plan.end());
    }
    return true;
}

bool MpcPathFollowerRos::isGoalReached()
{
    if (!isInitialized()) {
        ROS_ERROR("please call initialize() before using this planner");
        return false;
    }
    if (is_close_enough_) {
        ROS_INFO("Goal reached");
        return true;
    } else {
        return false;
    }
}

double MpcPathFollowerRos::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd MpcPathFollowerRos::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

void MpcPathFollowerRos::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
}

void MpcPathFollowerRos::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
}
};  // namespace mpc_path_follower
