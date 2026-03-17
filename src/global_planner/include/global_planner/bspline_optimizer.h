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

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d_ros.h>


// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point

namespace global_planner
{
  using namespace std;
  class BsplineOptimizer
  {

  public:
    static const int SMOOTHNESS;
    static const int ENDPOINT;
    static const int GUIDE;
    static const int WAYPOINTS;
    static const int CORRIDOR;
    static const int OBSTACLE;
    
    static const int GUIDE_PHASE;
    static const int NORMAL_PHASE;

    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &waypoints,const Eigen::MatrixXd &points,
                                        const std::vector<geometry_msgs::PolygonStamped> &corridors,
                                        costmap_2d::Costmap2D *costmap,
                                        const double &ts, const int &cost_function, int max_num_id,
                                        int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);

    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    void setCorridors(const std::vector<geometry_msgs::PolygonStamped> &corridors);
    void setParam();
    void setCostmap(costmap_2d::Costmap2D *costmap);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();
    vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd &ctrl_pts);
      void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                               const vector<Eigen::Vector3d> &start_end_derivative,
                                               Eigen::MatrixXd &ctrl_pts);
    void setPointsIndex(const Eigen::MatrixXd &points,
                                        std::vector<Eigen::Vector3d> &waypts, std::vector<int> &waypt_idx);
  private:
    // main input
    ros::NodeHandle nh_;
    Eigen::MatrixXd control_points_;    // B-spline control points, N x dim
    double bspline_interval_;           // B-spline knot span
    Eigen::Vector3d end_pt_;            // end of the trajectory
    int dim_;                           // dimension of the B-spline
                                        //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    bool dynamic_;                      // moving obstacles ?
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;      // bspline degree
    double lambda1_; // jerk smoothness weight
    double lambda2_; // end point weight
    double lambda3_; // guide cost weight
    double lambda4_; // waypoints cost weight
    double lambda5_; // corridor weight

    int algorithm1_; // optimization algorithms for quadratic cost
    int algorithm2_; // optimization algorithms for general cost
    // int max_iteration_num_[4];     // stopping criteria that can be used
    // double max_iteration_time_[4]; // stopping criteria that can be used

    /* intermediate variables */
    /* buffer for gradient of cost function, to avoid repeated allocation and
     * release of memory */
    vector<Eigen::Vector3d> g_q_;
    vector<Eigen::Vector3d> g_smoothness_;
    vector<Eigen::Vector3d> g_endpoint_;
    vector<Eigen::Vector3d> g_guide_;
    vector<Eigen::Vector3d> g_waypoints_;
    vector<Eigen::Vector3d> g_corridor_;
    vector<Eigen::Vector3d> g_obstacle_; // obstacle cost gradient

    int variable_num_;                  // optimization variables
    int iter_num_;                      // iteration of the solver
    std::vector<double> best_variable_; //
    double min_cost_;                   //

    vector<Eigen::Vector3d> block_pts_; // blocking points to compute visibility

    std::vector<geometry_msgs::PolygonStamped> corridors_; // safety corridors
    costmap_2d::Costmap2D *costmap_;                       // costmap pointer
    std::vector<std::vector<std::pair<double, double>>> var_bdk_; // [x,y]
    double curvature_constraint_sqr_; // curvature constraint square

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const vector<Eigen::Vector3d> &q, double &cost,
                            vector<Eigen::Vector3d> &gradient);
    // void calcFeasibilityCost(const vector<Eigen::Vector3d> &q, double &cost,
    //                          vector<Eigen::Vector3d> &gradient);
    void calcEndpointCost(const vector<Eigen::Vector3d> &q, double &cost,
                          vector<Eigen::Vector3d> &gradient);
    void calcObstacleCost(const vector<Eigen::Vector3d> &q, double &cost,
                          vector<Eigen::Vector3d> &gradient);
    void calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient);
    void calcCorridorCost(const vector<Eigen::Vector3d> &q, double &cost,vector<Eigen::Vector3d> &gradient);
    void calcVisibilityCost(const vector<Eigen::Vector3d> &q, double &cost,
                            vector<Eigen::Vector3d> &gradient);
    void calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                           vector<Eigen::Vector3d> &gradient);
    void calcViewCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient);
    bool isQuadratic();
    bool isObstacleInMap(const double x, const double y) const;
    void segmentPoints(const Eigen::MatrixXd &points, const Eigen::MatrixXd &waypoints,
                                       std::vector<Eigen::Vector3d> &waypts, std::vector<int> &waypt_idx);
    void calcNextYaw(const double &last_yaw, double &yaw);
    void setBsplineCorridor(const std::vector<geometry_msgs::PolygonStamped> &corridor);
    void calcMeanDistance();
    double getMapDistance(double wx, double wy);

    /* for benckmark evaluation only */
  public:
    vector<double> vec_cost_;
    vector<double> vec_time_;
    ros::Time time_start_;

    void getCostCurve(vector<double> &cost, vector<double> &time)
    {
      cost = vec_cost_;
      time = vec_time_;
    }

    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} 

#endif