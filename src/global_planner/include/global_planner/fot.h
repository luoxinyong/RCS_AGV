#ifndef GLOBAL_PLANNER_FRENET_PLANNER_H
#define GLOBAL_PLANNER_FRENET_PLANNER_H

#include <global_planner/planner_core.h>
#include "common_msgs/plc_res_nav_data.h"
#include <global_planner/expander.h> 
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <chrono>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <mutex> 
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath> // for std::mutex
namespace global_planner {

class QuinticPolynomial {
    public:
        // 构造函数，起点状态(d0, d0_dot, d0_ddot)，终点位置(dT)，规划时间(T)
        QuinticPolynomial(double d0, double d0_dot, double d0_ddot,
                        double dT, double T) {
            a0 = d0;
            a1 = d0_dot;
            a2 = d0_ddot / 2.0;

            double T2 = T * T;
            double T3 = T2 * T;
            double T4 = T3 * T;
            double T5 = T4 * T;

            Eigen::Matrix3d A;
            A << T3,     T4,      T5,
                3*T2,   4*T3,    5*T4,
                6*T,   12*T2,   20*T3;

            Eigen::Vector3d b;
            b << dT - a0 - a1*T - a2*T2,
                0.0 - a1 - 2*a2*T,
                0.0 - 2*a2;

            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
            a3 = x(0);
            a4 = x(1);
            a5 = x(2);
        }

        // 计算当前位置
        double calc_point(double t) const {
            double t2 = t*t, t3=t2*t, t4=t3*t, t5=t4*t;
            return a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        }

        // 一阶导数
        double calc_first_derivative(double t) const {
            double t2 = t*t, t3=t2*t, t4=t3*t;
            return a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
        }

        // 二阶导数
        double calc_second_derivative(double t) const {
            double t2 = t*t, t3=t2*t;
            return 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
        }

        // 三阶导数
        double calc_third_derivative(double t) const {
            double t2 = t*t;
            return 6*a3 + 24*a4*t + 60*a5*t2;
        }

    private:
        double a0, a1, a2, a3, a4, a5;
};

class QuinticLongitudinal {
public:
    // 构造函数
    // s0, s_dot0, s_ddot0 : 起点状态
    // sT                 : 终点位置
    // vT, aT             : 终点速度/加速度，如果不是最终目标可以传 NAN
    // T                  : 规划时间
    QuinticLongitudinal(double s0, double s_dot0, double s_ddot0,
                        double sT, double vT, double aT, double T) 
    {
        a0 = s0;
        a1 = s_dot0;
        a2 = s_ddot0 / 2.0;

        double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

        if (std::isnan(vT) && std::isnan(aT)) {
            // 中间局部目标，只约束位置 → 四次退化
            Eigen::Matrix2d A;
            A << T3, T4,
                 3*T2, 4*T3;
            Eigen::Vector2d b;
            b << sT - a0 - a1*T - a2*T2,
                  - a1 - 2*a2*T;  // 不约束速度，也可以保留0作为参考
            Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
            a3 = x(0); a4 = x(1);
            a5 = 0.0;
        } else {
            // 最终目标，约束位置 + 速度 + 加速度
            Eigen::Matrix3d A;
            A << T3,     T4,      T5,
                 3*T2,   4*T3,    5*T4,
                 6*T,   12*T2,   20*T3;
            Eigen::Vector3d b;
            b << sT - a0 - a1*T - a2*T2,
                 vT - a1 - 2*a2*T,
                 aT - 2*a2;
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
            a3 = x(0); a4 = x(1); a5 = x(2);
        }
    }

    double calc(double t) const {
        return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
    }

    double calc_d(double t) const {
        return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
    }

    double calc_dd(double t) const {
        return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
    }

    double calc_ddd(double t) const {
        return 6*a3 + 24*a4*t + 60*a5*t*t;
    }

private:
    double a0,a1,a2,a3,a4,a5;
};

class FrenetPlanner : public Expander {
public:
    FrenetPlanner(PotentialCalculator* p_calc, int xs, int ys, bool use_eight_connect = true);
    void setCostmap(costmap_2d::Costmap2D* costmap) ;
    bool calculatePotentials(unsigned char* costs, double start_x, double start_y,
                             double end_x, double end_y, int cycles, float* potential) override;
    void getpath( std::vector<geometry_msgs::PoseStamped>& plan);

private:
    
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void globalVelCallback(const common_msgs::plc_res_nav_data::ConstPtr& msg);
    bool generateTrajectory(double start_x, double start_y, double end_x, double end_y,
                            costmap_2d::Costmap2D* costmap,
                            std::vector<geometry_msgs::PoseStamped>& out_path);
    bool projectToReference(double robot_x, double robot_y, double& s0, double& d0, double& ref_yaw);
    bool isTrajectoryCollisionFree(const std::vector<double>& xs, const std::vector<double>& ys,
                                   const std::vector<double>& thetas, costmap_2d::Costmap2D* costmap,
                                   unsigned char lethal_cost);
    bool isCollisionWithFootprint(int x, int y, double direction, costmap_2d::Costmap2D* costmap,
                                  int nx, int ny, int ns, unsigned char lethal_cost, double resolution);

    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber global_vel_sub_;
    ros::Publisher traj_marker_pub_;
    ros::Publisher invalid_traj_marker_pub_;
    ros::Publisher traj_viz_pub_;


    // 参考路径
    std::vector<geometry_msgs::Point32> ref_points_;
    std::vector<geometry_msgs::PoseStamped> reference_poses_;  // 原始 PoseStamped
    costmap_2d::Costmap2D* costmap_ = nullptr;
    std::vector<double> cum_s_;
    double S_GOAL = 0.0;  // 全局终点 s
    bool has_reference_ = false;
    std::mutex path_mutex_;

    // 实时状态
    double vel_linear_ = 0.0;

    // 输出路径
    std::vector<geometry_msgs::PoseStamped> path_;

    // 超参数（可通过参数服务器加载）
    double MIN_T = 3.0;
    double MAX_T = 4.0;
    double DT = 0.2;
    double MAX_ROAD_WIDTH = 1.2;
    double D_ROAD_W = 0.4;
    double MAX_SPEED = 10;
    double MAX_ACCEL = 10;
    double MAX_CURVATURE = 1.5;

    double K_J = 0.1;//Jerk代价
    double K_T = 0.3;//时间代价
    double K_D = 1.0;//横向代价
    double K_S = 1.0;//终点停止代价
    double K_LAT = 1.0;//横向路径权重
    double K_LON = 1.0;//纵向路径权重，可以根据两类路径进行调整
};
}  // namespace global_planner

#endif  // GLOBAL_PLANNER_FRENET_PLANNER_H