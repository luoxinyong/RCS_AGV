#include "global_planner/fot.h"  // 您的头文件路径
#include <limits>  // 用于numeric_limits
#include <ros/ros.h>  // 用于ROS_WARN/ERROR，如果需要

namespace global_planner {



FrenetPlanner::FrenetPlanner(PotentialCalculator* p_calc, int xs, int ys, bool use_eight_connect)
    : Expander(p_calc, xs, ys), costmap_(nullptr) {
        global_path_sub_ = nh_.subscribe("/dense_path_show", 1, &FrenetPlanner::globalPathCallback, this);
        global_vel_sub_ = nh_.subscribe("/plc_res_nav_data", 1, &FrenetPlanner::globalVelCallback, this);
        traj_viz_pub_ = nh_.advertise<nav_msgs::Path>("/frenet/all_valid_paths", 1);
//         traj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("frenet_best_traj_marker", 1);
// invalid_traj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("frenet_last_invalid_traj_marker", 1);
        // 初始化
}

void FrenetPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg || msg->poses.empty()) {
        ROS_WARN("FrenetPlanner: Empty reference path received");
        return;
    }
    std::lock_guard<std::mutex> lock(path_mutex_);

    // 直接保存原始 PoseStamped
    reference_poses_ = msg->poses;
    ref_points_.clear();
    cum_s_.clear();
    cum_s_.push_back(0.0);

    for (const auto& pose : reference_poses_) {
        geometry_msgs::Point32 p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.0;
        ref_points_.push_back(p);
    }
       for (size_t i = 1; i < ref_points_.size(); ++i) {
        double dx = ref_points_[i].x - ref_points_[i-1].x;
        double dy = ref_points_[i].y - ref_points_[i-1].y;
        cum_s_.push_back(cum_s_.back() + std::hypot(dx, dy));
        }

    has_reference_ = true;
    S_GOAL = cum_s_.back();

}
void FrenetPlanner::globalVelCallback(const common_msgs::plc_res_nav_data::ConstPtr& msg) {
    vel_linear_ = msg->vel_linear;
}

void FrenetPlanner::getpath( std::vector<geometry_msgs::PoseStamped>& plan) {
    plan = path_;
}
void FrenetPlanner::setCostmap(costmap_2d::Costmap2D* costmap) {
    costmap_ = costmap;
}

bool FrenetPlanner::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    if (!costmap_ || reference_poses_.empty()) {
        ROS_ERROR("Costmap or reference points not set!");
        path_.clear();
        return false;
    }
    path_.clear();
    if (generateTrajectory(start_x, start_y, end_x, end_y, costmap_, path_)) {
        // 可根据path填充potential，或直接返回true（取决于您的框架）
        return true;
    }
    return false;
}


bool FrenetPlanner::generateTrajectory(double start_x, double start_y,
                            double end_x, double end_y, costmap_2d::Costmap2D* costmap, 
                            std::vector<geometry_msgs::PoseStamped>& out_path) {
    /////////////////////////////
    nav_msgs::Path all_valid_paths;
    all_valid_paths.header.frame_id = "map";
    all_valid_paths.header.stamp = ros::Time::now();
    //////////////////////////////
    int total ,speed ,accel ,curvatrue ,collision,cum = 0;
    if (!has_reference_ || ref_points_.size() < 2) return false;
    std::cout<<"start pose: "<<start_x<<", "<<start_y<<", end pose: "<<end_x<<", "<<end_y<<std::endl;
    // 1. 将起点投影到参考路径
    double s0 = 0.0, d0 = 0.0, ref_yaw = 0.0;
    if (!projectToReference(start_x, start_y, s0, d0, ref_yaw)) return false;

    double c_d = d0;
    double c_d_d = 0.0;
    double c_d_dd = 0.0;
    double c_s_d = vel_linear_;
    double c_s_dd = 0.0;

    // 2. 局部纵向目标规划（滑动窗口），每次规划 Δs
    double delta_s = 2.0;  // 每次规划纵向长度，可根据速度动态调整
    double s_goal_local = std::min(s0 + delta_s, S_GOAL);
    double a_target = 0.0; double v_target = 0.0;
    bool is_last_segment = (s_goal_local >= S_GOAL - 1e-3);
    
    double vT = is_last_segment ? 0.0 : NAN;
    double aT = is_last_segment ? 0.0 : NAN;
    double D_END_TARGET = 0.0;
    double MAX_END_OFFSET = 2.0;
 
    std::cout << "[FrenetPlanner] Start projection: s0=" << s0 << ", d0=" << d0 
              << ", ref_yaw=" << ref_yaw << ", c_s_d=" << c_s_d << std::endl;
    double best_cost = std::numeric_limits<double>::max();
    std::vector<std::pair<double, double>> best_traj;

    // 3. 遍历规划时间
    for (double Ti = MIN_T; Ti <= MAX_T; Ti += DT) {
        QuinticLongitudinal  lon_poly(s0, c_s_d, c_s_dd,s_goal_local, vT, aT, Ti);
        std::cout << "[FrenetPlanner] Sampling Ti=" << Ti << std::endl;
        // 4. 遍历横向偏移采样
        for (double di = -MAX_ROAD_WIDTH; di <= MAX_ROAD_WIDTH; di += D_ROAD_W) {
            total++;
            double desired_end_d = D_END_TARGET;  // 默认回 0
            if (std::abs(c_d) > 0.6) {
                // 已经偏了，就允许继续偏或缓慢回
                desired_end_d = c_d * 0.7;  // 往回拉 30%
            }
            desired_end_d = std::max(-MAX_END_OFFSET, std::min(MAX_END_OFFSET, desired_end_d));
            QuinticPolynomial lat_poly(c_d, c_d_d, c_d_dd, desired_end_d, Ti);

            bool valid = true;
            std::vector<double> xs, ys, thetas;
            double Jp = 0.0, Js = 0.0;

            // 5. 时间采样
            for (double t = 0.0; t <= Ti + 1e-6; t += DT) {
                double s = lon_poly.calc(t);
                double ds = lon_poly.calc_d(t);
                double dds = lon_poly.calc_dd(t);
                double s_ddd = lon_poly.calc_ddd(t);

                double d = lat_poly.calc_point(t);
                double dd = lat_poly.calc_first_derivative(t);
                double ddd = lat_poly.calc_second_derivative(t);
                double d_ddd = lat_poly.calc_third_derivative(t);

                Jp += d_ddd * d_ddd;
                Js += s_ddd * s_ddd;

                // 插值参考点
               double s_min = cum_s_.front();
                double s_max = cum_s_.back();

                if (s < s_min) {
                    std::cout << "[FrenetPlanner] s(" << s << ") < s_min(" << s_min << ") -> clamped\n";
                    s = s_min;
                }
                if (s > s_max) {
                    std::cout << "[FrenetPlanner] s(" << s << ") > s_max(" << s_max << ") -> clamped\n";
                    s = s_max;
                }

                const double EPS_S = 1e-6;
                if (std::abs(s - s_max) < 1e-9) {
                    // 回退到最后一个区间内部，便于线性插值
                    s = s_max - EPS_S;
                }

                auto it = std::lower_bound(cum_s_.begin(), cum_s_.end(), s);
                size_t idx = std::distance(cum_s_.begin(), it);
                if (idx == 0) idx = 1; // 使用第一个区间 [0,1]
                if (idx >= cum_s_.size()) idx = cum_s_.size() - 1; // 防越界

                double seg_len = (cum_s_[idx] - cum_s_[idx-1]);
                if (seg_len <= 1e-5) { valid = false;cum++; break; } // 避免除0

                double frac = (s - cum_s_[idx-1]) / seg_len;

                double x_r = ref_points_[idx-1].x + frac * (ref_points_[idx].x - ref_points_[idx-1].x);
                double y_r = ref_points_[idx-1].y + frac * (ref_points_[idx].y - ref_points_[idx-1].y);
                double dx = ref_points_[idx].x - ref_points_[idx-1].x;
                double dy = ref_points_[idx].y - ref_points_[idx-1].y;
                double theta_r = std::atan2(dy, dx);

                // Frenet -> Cartesian
                double x = x_r - d * std::sin(theta_r);
                double y = y_r + d * std::cos(theta_r);

                double d_d = dd / (ds + 1e-6);
                double d_dd = (ddd - d_d * dds) / ((ds + 1e-6) * (ds + 1e-6));
                double theta = theta_r + std::atan(d_d);
                double v = ds * std::sqrt(1.0 + d_d*d_d);
                double a = dds * std::sqrt(1.0 + d_d*d_d) + ds * d_d * d_dd / std::sqrt(1.0 + d_d*d_d);
                double kappa = d_dd / std::pow(1.0 + d_d*d_d, 1.5);

                // 速度/加速度/曲率检查
                if (v > MAX_SPEED || std::abs(a) > MAX_ACCEL || std::abs(kappa) > MAX_CURVATURE) {
                    valid = false; break;
                }
                if(v > MAX_SPEED) speed++;
                if(std::abs(a) > MAX_ACCEL) accel++;
                if(std::abs(kappa) > MAX_CURVATURE) curvatrue++;

                // 记录轨迹
                xs.push_back(x);
                ys.push_back(y);
                thetas.push_back(theta);
                // vs.push_back(v);
                // as.push_back(a);
                // kappas.push_back(kappa);
            }

            if (!valid) continue;
            if (!isTrajectoryCollisionFree(xs, ys, thetas, costmap, 0)) 
            {   
                collision++;
                continue;
            }
             {
                nav_msgs::Path sub_path;
                sub_path.header = all_valid_paths.header;

                for (size_t k = 0; k < xs.size(); k += 3) {  
                    // 每隔 3 个点记录一次，降低 RVIZ 压力
                    geometry_msgs::PoseStamped p;
                    p.header = sub_path.header;
                    p.pose.position.x = xs[k];
                    p.pose.position.y = ys[k];
                    p.pose.orientation.w = 1.0;
                    sub_path.poses.push_back(p);
                }

                // 将该子路径附加到总路径列表中
                all_valid_paths.poses.insert(all_valid_paths.poses.end(),
                                             sub_path.poses.begin(), sub_path.poses.end());
            }
            // 6. 代价函数（横向 + 纵向）
            double s_end = lon_poly.calc(Ti);
            double d_end = lat_poly.calc_point(Ti);
            double lat_cost = K_J * Jp + K_T * Ti + K_D * (d_end * d_end);
            double lon_cost = K_J * Js + K_T * Ti + K_S * ((s_goal_local - s_end) * (s_goal_local - s_end));
            double cf = K_LAT * lat_cost + K_LON * lon_cost;

            if (cf < best_cost) {
                best_cost = cf;
                best_traj.clear();
                for (size_t k = 0; k < xs.size(); ++k) best_traj.emplace_back(xs[k], ys[k]);
            }
        }
    }
std::cout<<"total: "<<total<<", speed: "<<speed<<", accel: "<<accel<<", curvatrue: "<<curvatrue<<", collision: "<<collision<<", cum: "<<cum<<std::endl;

    // 7. 输出轨迹
    out_path.clear();
    for (auto &p : best_traj) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = p.first;
        pose.pose.position.y = p.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        out_path.push_back(pose);
    }
    traj_viz_pub_.publish(all_valid_paths);
    return !out_path.empty();
}


bool FrenetPlanner::isTrajectoryCollisionFree(const std::vector<double>& xs, const std::vector<double>& ys,
                                              const std::vector<double>& thetas, costmap_2d::Costmap2D* costmap,
                                              unsigned char lethal_cost) {
    // 同前
    double mx,my;
    
    if (xs.size() != ys.size() || xs.size() != thetas.size()) {
        ROS_WARN("Trajectory points size mismatch!");
        return false;
    }

    // unsigned char* costs_map = costmap->getCharMap();
    // if (!costs_map) {
    //     ROS_ERROR("Costmap char map is null!");
    //     return false;
    // }

    int nx = costmap_->getSizeInCellsX();
    int ny = costmap_->getSizeInCellsY();
    int ns = nx * ny;
    double resolution = costmap_->getResolution();
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    for (size_t i = 0; i < xs.size(); ++i) {
        double x = xs[i];
        double y = ys[i];
        double theta = thetas[i];

        unsigned int  mx,my;
        if (!costmap_->worldToMap(x, y, mx, my)) {
            ROS_DEBUG("Point (%.3f, %.3f) is outside costmap bounds!", x, y);
            return false;
        }

        if (mx < 0 || mx >= nx || my < 0 || my >= ny) {
            return false;
        }

        if(costmap_->getCost(mx,my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE )
        {
            ROS_ERROR("the point (%.3f, %.3f) is in the obstacle", x, y);
            return  false;
        }

        if (isCollisionWithFootprint(mx, my, theta, costmap_, nx, ny, ns, lethal_cost, resolution)) {
            return false;
        }
    }

    return true;
}

bool FrenetPlanner::isCollisionWithFootprint(int x, int y, double direction, costmap_2d::Costmap2D* costmap, int nx, int ny, int ns,
                                             unsigned char lethal_cost, double resolution) {
    static const std::vector<std::pair<double, double>> footprint_ = {
        {0.0,   -0.5},
        {1.62,  -0.5},
        {1.62,   0.5},
        {0.0,    0.5}
    };
    
    if (!costmap) return true;

    // 1. 获取世界坐标（中心点）
    double wx, wy;
    costmap->mapToWorld(x, y, wx, wy);

    // 2. 旋转矩阵（yaw）
    double cos_t = std::cos(direction);
    double sin_t = std::sin(direction);

    // 3. 构建 footprint 在 map 坐标下的点
      std::vector<std::pair<int, int>> footprint_map; 
    footprint_map.reserve(footprint_.size());
    
    int nx_map = costmap->getSizeInCellsX();  // nx_map
    int ny_map = costmap->getSizeInCellsY();  // ny_map

    for (const auto& p : footprint_) {
        // p 是车体局部坐标（例如 [-0.25, -0.15], [0.25, -0.15], ...）

        // 车体坐标系 → 世界坐标
          double wx_f = wx + p.first * cos_t - p.second * sin_t;
        double wy_f = wy + p.first * sin_t + p.second * cos_t;

        // 世界坐标 → 地图坐标
        unsigned int mx_f, my_f;
        if (!costmap->worldToMap(wx_f, wy_f, mx_f, my_f)) {
            return true;  // footprint 掉出地图 → 认为碰撞
        }

        footprint_map.push_back({mx_f, my_f});
    }

    // 4. 遍历 footprint 边，检查每个栅格
    for (size_t i = 0; i < footprint_map.size(); i++) {
        const auto& p1 = footprint_map[i];
        const auto& p2 = footprint_map[(i + 1) % footprint_map.size()];

        int x0 = p1.first, y0 = p1.second;
        int x1 = p2.first, y1 = p2.second;

        // Bresenham 线扫描算法
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0, y = y0;

        while (true) {
            if (x < 0 || x >= nx_map || y < 0 || y >= ny_map)
                return true;

            if (costmap->getCost(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                return true;

            if (x == x1 && y == y1)
                break;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
    }

    // 5. 检查 footprint 内部点（可选但强烈推荐）
    for (const auto& p : footprint_map) {
        if (costmap->getCost(p.first, p.second) >= lethal_cost) {
            return true;
        }
    }

    return false;
}
bool FrenetPlanner::projectToReference(double robot_x, double robot_y,
                                       double& s0, double& d0, double& ref_yaw)
{

    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!has_reference_) return false;
    // 使用最近线段投影：遍历每段，取距离最小的投影点
    double best_s = 0.0; double best_d = 0.0; double best_yaw = 0.0;
    double best_dist2 = std::numeric_limits<double>::max();
    double acc_s = 0.0;
    for (size_t i = 0; i+1 < ref_points_.size(); ++i) {
    Eigen::Vector2d p0(ref_points_[i].x, ref_points_[i].y);
    Eigen::Vector2d p1(ref_points_[i+1].x, ref_points_[i+1].y);
    Eigen::Vector2d v = p1 - p0;
    double seg_len2 = v.squaredNorm();
    if (seg_len2 < 1e-8) continue;
    Eigen::Vector2d rp(robot_x - p0.x(), robot_y - p0.y());
    double t = rp.dot(v) / seg_len2;
    double t_clamped = std::min(1.0, std::max(0.0, t));
    Eigen::Vector2d proj = p0 + v * t_clamped;
    double d2 = (proj - Eigen::Vector2d(robot_x, robot_y)).squaredNorm();
    if (d2 < best_dist2) {
    best_dist2 = d2;
    double seg_s = std::hypot(v.x(), v.y()) * t_clamped;
    best_s = acc_s + seg_s;
    // 横向 d：点到线的有向距离，法向量为 (-vy, vx)
    Eigen::Vector2d n(-v.y(), v.x());
    n.normalize();
    Eigen::Vector2d diff(robot_x - proj.x(), robot_y - proj.y());
    best_d = diff.dot(n);
    best_yaw = std::atan2(v.y(), v.x());
    }
    acc_s += std::sqrt(seg_len2);
    }
    s0 = best_s; d0 = best_d; ref_yaw = best_yaw;
    return true;

}
}  // namespace global_planner