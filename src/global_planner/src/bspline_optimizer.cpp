/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include "global_planner/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;


namespace global_planner
{

  const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
  const int BsplineOptimizer::ENDPOINT = (1 << 1);
  const int BsplineOptimizer::GUIDE = (1 << 2);
  const int BsplineOptimizer::WAYPOINTS = (1 << 3);
  const int BsplineOptimizer::CORRIDOR = (1 << 4);
  const int BsplineOptimizer::OBSTACLE = (1 << 5);

  const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE | BsplineOptimizer::CORRIDOR;
  const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::ENDPOINT | BsplineOptimizer::WAYPOINTS;

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    std::vector<Eigen::Vector3d> points_vec;
    for (int i = 0; i < points.rows(); ++i)
    {
      points_vec.emplace_back(points(i, 0), points(i, 1), points(i, 2));
    }
    std::vector<Eigen::Vector3d> start_end_determinant = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    double ts = 0.2;
    parameterizeToBspline(ts, points_vec, start_end_determinant, control_points_);
    dim_ = control_points_.cols();
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setTerminateCond(const int &max_num_id, const int &max_time_id)
  {
    max_num_id_ = max_num_id;
    max_time_id_ = max_time_id;
  }

  void BsplineOptimizer::setCorridors(const std::vector<geometry_msgs::PolygonStamped> &corridors)
  {
    corridors_ = corridors;
  }

  void BsplineOptimizer::setParam()
  {
    lambda1_ = 10.0; //Smooth
    lambda2_ = 100.0; //obstacle
    lambda3_ = 5.0; //guide
    lambda4_ = 100.0; //waypoint
    lambda5_ = 40.0; //safety_corr
    order_ = 3;
    algorithm1_ = nlopt::LD_LBFGS;  // for quadratic cost
    algorithm2_ = nlopt::LN_BOBYQA; // for non-quadratic cost
  }

 void BsplineOptimizer::setCostmap(costmap_2d::Costmap2D *costmap)
  {
    costmap_ = costmap;
  }

  void BsplineOptimizer::setCostFunction(const int &cost_code)
  {
    cost_function_ = cost_code;

    // print optimized cost function
    string cost_str;
    if (cost_function_ & SMOOTHNESS)
      cost_str += "smooth |";
    if (cost_function_ & ENDPOINT)
      cost_str += " endpt |";
    if (cost_function_ & GUIDE)
      cost_str += " guide |";
    if (cost_function_ & WAYPOINTS)
      cost_str += " waypt |";
    if (cost_function_ & CORRIDOR)
      cost_str += " corridor |";

    ROS_INFO_STREAM("cost func: " << cost_str);
  }

  void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> &guide_pt) { guide_pts_ = guide_pt; }

  void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d> &waypts,
                                      const vector<int> &waypt_idx)
  {
    waypoints_ = waypts;
    waypt_idx_ = waypt_idx;
  }
void BsplineOptimizer::setPointsIndex(const Eigen::MatrixXd &points,
                                        std::vector<Eigen::Vector3d> &waypts, std::vector<int> &waypt_idx)
  {
    waypts.clear();
    waypt_idx.clear();

    // 每个point的索引
    for (int i = 0; i < points.rows(); ++i)
    {
      waypts.emplace_back(points(i, 0), points(i, 1), points(i, 2));
      waypt_idx.emplace_back(i);
    }
  }

  Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd &waypoints,const Eigen::MatrixXd &points,
                                                        const std::vector<geometry_msgs::PolygonStamped> &corridors,
                                                        costmap_2d::Costmap2D *costmap,
                                                        const double &ts, const int &cost_function, int max_num_id,
                                                        int max_time_id)
  {
          

    setParam();
    setCorridors(corridors);
    setCostmap(costmap);
    setControlPoints(waypoints);
    std::vector<Eigen::Vector3d> waypts;
    std::vector<int> waypt_idx;
    setPointsIndex(points, waypts, waypt_idx);//control_points_
  
    setWaypoints(waypts, waypt_idx);
    setBsplineCorridor(corridors);

    setBsplineInterval(ts);
    setCostFunction(cost_function);
    setTerminateCond(max_num_id, max_time_id);

    optimize();
    return this->control_points_;
      //   vector<Eigen::Vector3d> waypoints;
    //   waypoints.reserve(waypoints_mat.rows());

    //   for (int i = 0; i < waypoints_mat.rows(); ++i) {
    //       Eigen::Vector3d p(0,0,0);

    //       // 如果你是 N×2，则第三维补 0
    //       p.x() = waypoints_mat(i, 0);
    //       p.y() = waypoints_mat(i, 1);

    //       if (waypoints_mat.cols() >= 3)
    //           p.z() = waypoints_mat(i, 2);

    //       waypoints.push_back(p);
    //   }

    // for (auto &wp : waypoints) {

    //       // 找到距离最近的控制点 q[k]
    //       double best = 1e9;
    //       int best_idx = 0;

    //       for (int k = 0; k < control_points_.rows() - 2; k++) {
    //           Eigen::Vector3d mid =
    //         (control_points_.row(k) +
    //          control_points_.row(k+1) +
    //          control_points_.row(k+2)) / 3.0;
    //           double d = (wp - mid).squaredNorm();
    //           if (d < best) {
    //               best = d;
    //               best_idx = k;
    //           }
    //       }

    //       waypt_idx_.push_back(best_idx);
    //   }
    // setWaypoints(waypoints, waypt_idx_);

    // 计算waypoints的yaw
  }

  void BsplineOptimizer::optimize()
  {
    /* initialize solver */
    iter_num_ = 0;
    min_cost_ = std::numeric_limits<double>::max();
    const int pt_num = control_points_.rows();
    g_q_.resize(pt_num);
    g_smoothness_.resize(pt_num);
    g_endpoint_.resize(pt_num);
    g_waypoints_.resize(pt_num);
    g_guide_.resize(pt_num);
    g_corridor_.resize(pt_num);
    g_obstacle_.resize(pt_num);

    if (cost_function_ & ENDPOINT)
    {
      variable_num_ = dim_ * (pt_num - order_);
      // end position used for hard constraint
      end_pt_ = (1 / 6.0) *
                (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
                 control_points_.row(pt_num - 1));
    }
    else
    {
      variable_num_ = max(0, dim_ * (pt_num - 2 * order_));
    }
    std::cout<<"dim: "<<dim_<<std::endl;
    /* do optimization using NLopt slover */
    nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
    opt.set_min_objective(BsplineOptimizer::costFunction, this);
    opt.set_maxeval(max_num_id_);
    opt.set_maxtime(max_time_id_);
    opt.set_xtol_rel(1e-5);

    vector<double> q(variable_num_);
    for (int i = order_; i < pt_num; ++i)
    {
      if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
        continue;
      for (int j = 0; j < dim_; j++)
      {
        q[dim_ * (i - order_) + j] = control_points_(i, j);
      }
    }

    if (dim_ != 1)
    {
      vector<double> lb(variable_num_), ub(variable_num_);
      const double bound = 10.0;
      for (int i = 0; i < variable_num_; ++i)
      {
        lb[i] = q[i] - bound;
        ub[i] = q[i] + bound;
      }
      opt.set_lower_bounds(lb);
      opt.set_upper_bounds(ub);
    }

    try
    {
      // cout << fixed << setprecision(7);
      // vec_time_.clear();
      // vec_cost_.clear();
      // time_start_ = ros::Time::now();

      double final_cost;
      nlopt::result result = opt.optimize(q, final_cost);

      /* retrieve the optimization result */
      cout << "Min cost:" << min_cost_ << endl;
    }
    catch (std::exception &e)
    {
      ROS_WARN("[Optimization]: nlopt exception");
      cout << e.what() << endl;
    }

    for (int i = order_; i < control_points_.rows(); ++i)
    {
      if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
        continue;
      for (int j = 0; j < dim_; j++)
      {
        control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
      }
    }

    if (!(cost_function_ & GUIDE))
      ROS_INFO_STREAM("iter num: " << iter_num_);
  }


void BsplineOptimizer::calcObstacleCost(const vector<Eigen::Vector3d> &q, double &cost,
                                          vector<Eigen::Vector3d> &gradient)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    // double dist;
    // Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

    const double safe_dist = 0.5;   // 安全阈值(m)
    const double eps = 1e-3;        // 数值梯度步长
for (int k = 0; k < 2; k++){
  for (size_t i = 0; i < q.size(); ++i)
    {
        double dist = getMapDistance(q[i].x(), q[i].y()); // 距离障碍的欧氏距离
        if (dist < safe_dist)
        {
            double diff = safe_dist - dist;
            cost += diff * diff; // 二次惩罚

            // 数值梯度近似
            double dx = (getMapDistance(q[i].x() + eps, q[i].y()) - dist) / eps;
            double dy = (getMapDistance(q[i].x(), q[i].y() + eps) - dist) / eps;

            gradient[i](k)  -= 2.0 * diff * dx;
            gradient[i](k)  -= 2.0 * diff * dy;
        }
    }
  }
    
  }

  void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d> &q, double &cost,
                                            vector<Eigen::Vector3d> &gradient)
  {
    const double curvature_weight = 1.0;
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);
    Eigen::Vector3d jerk, temp_j;
    

    for (int i = 0; i < q.size() - order_; i++)
    {
      /* evaluate jerk */
      jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
      cost += jerk.squaredNorm();
      temp_j = 2.0 * jerk;
      /* jerk gradient */
      gradient[i + 0] += -temp_j;
      gradient[i + 1] += 3.0 * temp_j;
      gradient[i + 2] += -3.0 * temp_j;
      gradient[i + 3] += temp_j;

      // evaluate curvature
      Eigen::Vector3d temp_c = q[i] - 2.0 * q[i + 1] + q[i + 2];
      double curvature_max_bound = temp_c.squaredNorm() - curvature_constraint_sqr_;
      if (curvature_max_bound > 0.0)
      {
        cost += curvature_max_bound * curvature_max_bound * curvature_weight;

        Eigen::Vector3d dcost_dtemp_c = 2.0 * curvature_max_bound * 2.0 * temp_c * curvature_weight;
        gradient[i + 0] += 1.0 * dcost_dtemp_c;
        gradient[i + 1] += -2.0 * dcost_dtemp_c;
        gradient[i + 2] += 1.0 * dcost_dtemp_c;
      }
    }
  }

  void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d> &q, double &cost,
                                          vector<Eigen::Vector3d> &gradient)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q[q.size() - 3];
    q_2 = q[q.size() - 2];
    q_1 = q[q.size() - 1];

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
    cost += dq.squaredNorm();

    gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
    gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
    gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                                           vector<Eigen::Vector3d> &gradient)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    Eigen::Vector3d q1, q2, q3, dq;

    // for (auto wp : waypoints_) {
    for (int i = 0; i < waypoints_.size(); ++i)
    {
      Eigen::Vector3d waypt = waypoints_[i];
      int idx = waypt_idx_[i];

      q1 = q[idx];
      q2 = q[idx + 1];
      q3 = q[idx + 2];

      dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
      cost += dq.squaredNorm();

      gradient[idx] += dq * (2.0 / 6.0);     // 2*dq*(1/6)
      gradient[idx + 1] += dq * (8.0 / 6.0); // 2*dq*(4/6)
      gradient[idx + 2] += dq * (2.0 / 6.0);
    }
  }

  /* use the uniformly sampled points on a geomertic path to guide the
   * trajectory. For each control points to be optimized, it is assigned a
   * guiding point on the path and the distance between them is penalized */
  void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost,
                                       vector<Eigen::Vector3d> &gradient)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    int end_idx = q.size() - order_;

    for (int i = order_; i < end_idx; i++)
    {
      Eigen::Vector3d gpt = guide_pts_[i - order_];
      cost += (q[i] - gpt).squaredNorm();
      gradient[i] += 2 * (q[i] - gpt);
    }
  }

  void BsplineOptimizer::calcCorridorCost(
    const vector<Eigen::Vector3d> &q,
    double &cost,
    vector<Eigen::Vector3d> &gradient)
{
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    double demarcation = 1.0;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    /*** calculate distance cost and gradient ***/
    for (int k = 0; k < 2; k++) // 处理前两个维度(x,y)
    {
      for (int i = 0; i < q.size() - 2; i++) // 遍历控制点窗口
      {
        for (int j = 0; j < order_; j++)
        {
          double upbound = 0, lowbound = 0;
          int point_idx = i + j;
          int polygon_idx = 3 * i + j;
          if (polygon_idx >= var_bdk_[k].size())
            continue;

          upbound = var_bdk_[k][polygon_idx].second;
          lowbound = var_bdk_[k][polygon_idx].first;
          double boxup = upbound - q[point_idx](k);
          double boxlow = q[point_idx](k) - lowbound;
          double updist_err = demarcation - boxup;
          double lowdist_err = demarcation - boxlow;

          if (updist_err >= 0)
          {
            if (updist_err < demarcation)
            {
              cost += pow(updist_err, 3);
              gradient[point_idx](k) += 3.0 * updist_err * updist_err;
            }
            else
            {
              cost += a * updist_err * updist_err + b * updist_err + c;
              gradient[point_idx](k) += 2.0 * a * updist_err + b;
            }
          }
          else if (lowdist_err >= 0)
          {
            if (lowdist_err < demarcation)
            {
              cost += pow(lowdist_err, 3);
              gradient[point_idx](k) += -3.0 * lowdist_err * lowdist_err;
            }
            else
            {
              cost += a * lowdist_err * lowdist_err + b * lowdist_err + c;
              gradient[point_idx](k) += -2.0 * a * lowdist_err - b;
            }
          }
        }
      }
    }
}


  void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                     double &f_combine)
  {
    /* convert the NLopt format vector to control points. */

    // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
    // For 1D case, the second and third elements are zero, and similar for the 2D case.
    for (int i = 0; i < order_; i++)
    {
      for (int j = 0; j < dim_; ++j)
      {
        g_q_[i][j] = control_points_(i, j);
      }
      for (int j = dim_; j < 3; ++j)
      {
        g_q_[i][j] = 0.0;
      }
    }

    for (int i = 0; i < variable_num_ / dim_; i++)
    {
      for (int j = 0; j < dim_; ++j)
      {
        g_q_[i + order_][j] = x[dim_ * i + j];
      }
      for (int j = dim_; j < 3; ++j)
      {
        g_q_[i + order_][j] = 0.0;
      }
    }

    if (!(cost_function_ & ENDPOINT))
    {
      for (int i = 0; i < order_; i++)
      {

        for (int j = 0; j < dim_; ++j)
        {
          g_q_[order_ + variable_num_ / dim_ + i][j] =
              control_points_(control_points_.rows() - order_ + i, j);
        }
        for (int j = dim_; j < 3; ++j)
        {
          g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
        }
      }
    }

    f_combine = 0.0;
    grad.resize(variable_num_);
    fill(grad.begin(), grad.end(), 0.0);

    /*  evaluate costs and their gradient  */
    double f_smoothness, f_endpoint, f_guide, f_waypoints, f_corridor;
    f_smoothness = f_endpoint = f_guide = f_waypoints = f_corridor = 0.0;

    if (cost_function_ & SMOOTHNESS)
    {
      calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
      f_combine += lambda1_ * f_smoothness;
      std::cout<<"smoothness cost: "<<lambda1_ * f_smoothness<<std::endl;
      for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
    }
    if (cost_function_ & ENDPOINT)
    {
      calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
      f_combine += lambda2_ * f_endpoint;
      for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += lambda2_ * g_endpoint_[i + order_](j);
    }
    if (cost_function_ & GUIDE)
    {
      calcGuideCost(g_q_, f_guide, g_guide_);
      f_combine += lambda3_ * f_guide;
      for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += lambda3_ * g_guide_[i + order_](j);
    }
    if (cost_function_ & WAYPOINTS)
    {
      calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
      f_combine += lambda4_ * f_waypoints;
      std::cout<<"waypoints cost: "<<lambda4_ * f_waypoints<<std::endl;
      for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += lambda4_ * g_waypoints_[i + order_](j);
    }
    if (cost_function_ & CORRIDOR)
    {
      calcCorridorCost(g_q_, f_corridor, g_corridor_);
      f_combine += lambda5_ * f_corridor;
      std::cout<<"corridor cost: "<<lambda5_ * f_corridor<<std::endl;
      for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += lambda5_ * g_corridor_[i + order_](j);
    }
    if (cost_function_ & OBSTACLE)
    {
      double f_obstacle = 0.0;
      calcObstacleCost(g_q_, f_obstacle, g_obstacle_);
      f_combine += lambda2_ * f_obstacle;
      std::cout<<"obstacle cost: "<<lambda2_ * f_obstacle<<std::endl;
      // for (int i = 0; i < variable_num_ / dim_; i++)
      //   for (int j = 0; j < dim_; j++)
      //     grad[dim_ * i + j] += lambda2_ * g_obstacle_[i + order_](j);
    }
    
    /*  print cost  */
    // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
    //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
    //        << ", waypt: " << lambda7_ * f_waypoints << endl;
    // }

    // if (optimization_phase_ == SECOND_PHASE) {
    //  << ", smooth: " << lambda1_ * f_smoothness
    //  << " , dist:" << lambda2_ * f_distance
    //  << ", fea: " << lambda3_ * f_feasibility << endl;
    // << ", end: " << lambda4_ * f_endpoint
    // << ", guide: " << lambda5_ * f_guide
    // }
  }

  double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                        void *func_data)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);
    opt->iter_num_++;

    /* save the min cost result */
    if (cost < opt->min_cost_)
    {
      opt->min_cost_ = cost;
      opt->best_variable_ = x;
    }
    return cost;

    // /* evaluation */
    // ros::Time te1 = ros::Time::now();
    // double time_now = (te1 - opt->time_start_).toSec();
    // opt->vec_time_.push_back(time_now);
    // if (opt->vec_cost_.size() == 0)
    // {
    //   opt->vec_cost_.push_back(f_combine);
    // }
    // else if (opt->vec_cost_.back() > f_combine)
    // {
    //   opt->vec_cost_.push_back(f_combine);
    // }
    // else
    // {
    //   opt->vec_cost_.push_back(opt->vec_cost_.back());
    // }
  }

  vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd &ctrl_pts)
  {
    vector<Eigen::Vector3d> ctrl_q;
    for (int i = 0; i < ctrl_pts.rows(); ++i)
    {
      ctrl_q.push_back(ctrl_pts.row(i));
    }
    return ctrl_q;
  }

  Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

  bool BsplineOptimizer::isQuadratic()
  {
    if (cost_function_ == GUIDE_PHASE)
    {
      return true;
    }
    else if (cost_function_ == (SMOOTHNESS | WAYPOINTS))
    {
      return true;
    }
    return false;
  }
  bool BsplineOptimizer::isObstacleInMap(const double x, const double y) const
  {
    unsigned int mx, my;

    costmap_->worldToMap(x, y, mx, my);

    if (mx >= costmap_->getSizeInCellsX() ||
        my >= costmap_->getSizeInCellsY())
    {
      return true; // Consider out-of-bounds as obstacle
    }

    unsigned char cost = costmap_->getCost(mx, my);
    return (cost == costmap_2d::LETHAL_OBSTACLE ||
            cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  void BsplineOptimizer::segmentPoints(const Eigen::MatrixXd &points, const Eigen::MatrixXd &waypoints,
                                       std::vector<Eigen::Vector3d> &waypts, std::vector<int> &waypt_idx)
  {
    waypts.clear();
    waypt_idx.clear();
    double last_yaw = 0.0;
    std::vector<int> split_indexes;
    std::vector<int> idx_num;
    if (points.rows() == 0 || waypoints.rows() == 0)
    {
      ROS_WARN("points or waypoints is empty, cannot segment points!");
      return;
    }

    split_indexes.push_back(0);

    // 找到每个point在waypoints中的索引
    for (int i = 0; i < points.rows(); ++i)
    {
      bool found = false;
      for (int j = 0; j < waypoints.rows(); ++j)
      {
        if (waypoints.row(j).isApprox(points.row(i)))
        {
          split_indexes.push_back(j);
          found = true;
          break;
        }
      }
      if (!found)
      {
        ROS_WARN("point %d not found in waypoints!", i);
        return;
      }
    }
    split_indexes.push_back(waypoints.rows() - 1);
    for (auto &idx : split_indexes)
    {
      if (idx < 0) idx = 0;
      if (idx >= waypoints.rows()) idx = waypoints.rows() - 1;
    }

    int seg_num = split_indexes.size() - 1;
    for (int i = 0; i < seg_num; ++i)
    {
      int num = split_indexes[i + 1] - split_indexes[i];
      if (num <= 0)
        continue;
      idx_num.push_back(num);
    }

    for (int i = 0; i < seg_num; ++i)
    {
      for (int j = 0; j < idx_num[i] - 1; ++j)
      {
        int idx_pc = split_indexes[i] + j;
        int idx_pf = idx_pc + 1;
        if (idx_pc >= waypoints.rows() || idx_pf >= waypoints.rows()) {
            ROS_WARN("Skipping segment i=%d, j=%d due to index out of bounds", i, j);
            continue;
        } 
        Eigen::Vector3d pc = waypoints.row(idx_pc);
        Eigen::Vector3d pf = waypoints.row(idx_pf);
        Eigen::Vector3d pd = pf - pc;

        Eigen::Vector3d waypt;
        if (pd.norm() > 1e-6)
        {
          waypt(0) = atan2(pd(1), pd(0));
          waypt(1) = waypt(2) = 0.0;
          calcNextYaw(last_yaw, waypt(0));
          last_yaw = waypt(0);
        }
        else if (!waypts.empty())
        {
          waypt = waypts.back();
        }
        else
        {
          waypt.setZero();
        }
        waypts.push_back(waypt);
        waypt_idx.push_back(split_indexes[i] + j);
      }
    }
  }
  void BsplineOptimizer::calcNextYaw(const double &last_yaw, double &yaw)
  {
    // round yaw to [-PI, PI]

    double round_last = last_yaw;

    while (round_last < -M_PI)
    {
      round_last += 2 * M_PI;
    }
    while (round_last > M_PI)
    {
      round_last -= 2 * M_PI;
    }

    double diff = yaw - round_last;

    if (fabs(diff) <= M_PI)
    {
      yaw = last_yaw + diff;
    }
    else if (diff > M_PI)
    {
      yaw = last_yaw + diff - 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
      yaw = last_yaw + diff + 2 * M_PI;
    }
  }

  void BsplineOptimizer::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                               const vector<Eigen::Vector3d> &start_end_derivative,
                                               Eigen::MatrixXd &ctrl_pts)
  {
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() < 2)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
    // cout << "A:\n" << A << endl;

    // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
    // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
    // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
    // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    ctrl_pts.resize(K + 2, 3);
    ctrl_pts.col(0) = px;
    ctrl_pts.col(1) = py;
    ctrl_pts.col(2) = pz;

    // cout << "[B-spline]: parameterization ok." << endl;
  }
  void BsplineOptimizer::setBsplineCorridor(const std::vector<geometry_msgs::PolygonStamped> &corridor)
  {
    var_bdk_.clear();
    var_bdk_.resize(2); // [x,y]

    for (int k = 0; k < static_cast<int>(corridor.size()); ++k)
    {
      const auto &poly = corridor[k];
      double scale_k = 1.0;

      // 提取所有点
      const auto &pts = poly.polygon.points;
      if (pts.empty())
        continue;

      // 计算多边形包围盒
      double xmin = std::numeric_limits<double>::max();
      double xmax = -std::numeric_limits<double>::max();
      double ymin = std::numeric_limits<double>::max();
      double ymax = -std::numeric_limits<double>::max();

      for (const auto &p : pts)
      {
        if (p.x < xmin)
          xmin = p.x;
        if (p.x > xmax)
          xmax = p.x;
        if (p.y < ymin)
          ymin = p.y;
        if (p.y > ymax)
          ymax = p.y;
      }

      // 每个维度 (x, y)
      for (int i = 0; i < 2; ++i)
      {
          std::pair<double, double> vb_x;

          double lo_bound = (i == 0 ? xmin : ymin) / scale_k;
          double up_bound = (i == 0 ? xmax : ymax) / scale_k;

          vb_x = std::make_pair(lo_bound, up_bound);
          var_bdk_[i].push_back(vb_x);
      }
    }
  }
   void BsplineOptimizer::calcMeanDistance()
  {
    double average_interval_length_ = 0.0;
    double total_length = 0.0;
    double max_curvature_ = 0.1; // 最大曲率

    for (int i = 1; i < control_points_.rows(); ++i)
    {
      Eigen::Vector3d p0 = control_points_.row(i - 1).transpose();
      Eigen::Vector3d p1 = control_points_.row(i).transpose();
      total_length += (p1 - p0).norm();
    }

    average_interval_length_ = total_length / (control_points_.rows() - 1);
    // double interval_sqr = average_interval_length_ * average_interval_length_;
    const double L8 = std::pow(average_interval_length_, 8);
    // curvature_constraint_sqr = (interval_sqr * max_curvature_) *
    //                            (interval_sqr * max_curvature_);
    curvature_constraint_sqr_ = (max_curvature_ * max_curvature_) * L8;
  }

  double BsplineOptimizer::getMapDistance(double wx, double wy)
{
    // 1. world -> map 坐标
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my))
        return std::numeric_limits<double>::infinity(); // 超出地图返回∞

    // 2. 获取代价图数据
    const unsigned char *costs = costmap_->getCharMap();
    int size_x = costmap_->getSizeInCellsX();
    int size_y = costmap_->getSizeInCellsY();

    // 3. 若无预生成距离场，则在局部窗口上计算最近障碍距离
    const double resolution = costmap_->getResolution();
    const int search_radius = 10; // 搜索窗口大小 (cell)

    double min_dist = std::numeric_limits<double>::infinity();
    for (int dx = -search_radius; dx <= search_radius; ++dx)
    {
        for (int dy = -search_radius; dy <= search_radius; ++dy)
        {
            int nx = static_cast<int>(mx) + dx;
            int ny = static_cast<int>(my) + dy;
            if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y)
                continue;

            unsigned char c = costs[ny * size_x + nx];
            if (c >= costmap_2d::LETHAL_OBSTACLE) // 碰到障碍
            {
                double dist = std::hypot(dx, dy) * resolution;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }
    }

    return min_dist == std::numeric_limits<double>::infinity() ? 5.0 : min_dist;
}
}  // namespace global_planner
