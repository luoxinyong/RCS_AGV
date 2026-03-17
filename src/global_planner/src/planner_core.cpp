/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/fot.h>

#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        bool use_eight_connet;
        private_nh.param("use_eight_connet",use_eight_connet,false);
        bool use_fot;
        private_nh.param("use_fot", use_fot, false);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
            std::cout<<"use_dijkstra"<<std::endl;
        }
        else if(use_fot)
        {
            fot_ = new FrenetPlanner(p_calc_, cx, cy,use_eight_connet);
            fot_->setCostmap(costmap_);
            planner_ = fot_;
            std::cout<<"use_fot"<<std::endl;
        }
        else
        {
            planner_ = new AStarExpansion(p_calc_, cx, cy,use_eight_connet);
            std::cout<<"use_astar"<<std::endl;
        }


        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        smooth_plan_pub_ = private_nh.advertise<nav_msgs::Path>("smooth_plan", 1);
        corridor_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("safety_corridors", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("curvature_threshold", curvature_threshold_, 0.0);
        private_nh.param("BsplineOptimizer", BsplineOptimizer_, false);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);
        
        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        initializeCorridorGenerator();  
        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}
void GlobalPlanner::initializeCorridorGenerator() {
    double safety_range = 0.5;  // 安全距离 [m]
    int max_iteration = 100;    // 最大迭代次数
    
    safety_corridor_ = std::make_unique<SafetyCorridor>(costmap_, safety_range, max_iteration);
    
}

// 在生成路径后调用
void GlobalPlanner::publishSafetyCorridors(const std::vector<geometry_msgs::PoseStamped>&  path) {
    if (!safety_corridor_) return;
    std::vector<geometry_msgs::PoseStamped> pruned_waypoints_;
    if (safety_corridor_->generateCorridors(path, frame_id_,corridors_, pruned_waypoints_)) {
        for (const auto& corridor : corridors_) {
            corridor_pub_.publish(corridor);
        }
        ROS_INFO("Published %zu safety corridors", corridors_.size());
    }
}
void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped temp_goal_pose;
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();
    smooth_plan_.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }


    double wx = start.pose.position.x;//起点坐标x
    double wy = start.pose.position.y;//起点坐标y

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    geometry_msgs::PoseStamped temp_goal = goal;
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Try a temp goal...");
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();
        double max_x = origin_x + costmap_->getSizeInCellsX() * costmap_->getResolution();
        double max_y = origin_y + costmap_->getSizeInCellsY() * costmap_->getResolution();

        double dx = goal.pose.position.x - start.pose.position.x;
        double dy = goal.pose.position.y - start.pose.position.y;

        double scale = 1.0;
        if (dx > 0) scale = std::min(scale, (max_x - start.pose.position.x) / dx);
        if (dx < 0) scale = std::min(scale, (origin_x - start.pose.position.x) / dx);
        if (dy > 0) scale = std::min(scale, (max_y - start.pose.position.y) / dy);
        if (dy < 0) scale = std::min(scale, (origin_y - start.pose.position.y) / dy);

        temp_goal.pose.position.x = start.pose.position.x + dx * scale;
        temp_goal.pose.position.y = start.pose.position.y + dy * scale;
        temp_goal.pose.position.z = 0.0;

        ROS_WARN("Goal is outside costmap, using temporary goal (%.2f, %.2f)", 
                 temp_goal.pose.position.x, temp_goal.pose.position.y);
        if (!costmap_->worldToMap(temp_goal.pose.position.x, temp_goal.pose.position.y,
                              goal_x_i, goal_y_i)) {
        ROS_ERROR("Temp goal still outside costmap! Planning will fail.");
        return false;
        }
    }


    if(costmap_->getCost(goal_x_i,goal_y_i) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || costmap_->getCost(goal_x_i,goal_y_i) == costmap_2d::LETHAL_OBSTACLE)
    {
        ROS_ERROR("the goal point is in the obstacle");
        return  false;
    }

    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(temp_goal.pose.position.x, temp_goal.pose.position.y, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    if(use_fot_){
         found_legal = fot_->calculatePotentials(costmap_->getCharMap(), start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y,
                                                    nx * ny * 2, potential_array_);
    }else{
         found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);
    }
    

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if(use_fot_)
        {
            fot_->getpath(plan);
            publishPlan(plan);
            publishSafetyCorridors(plan);
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        }else if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            publishPlan(plan);
            publishSafetyCorridors(plan);
            if (enable_simple_curvature_optimization_ && plan.size() > 3) {
                ROS_INFO("Applying simple curvature optimization");
                plan = simpleCurvatureOptimization(plan, curvature_threshold_);
                smooth_plan_  = plan;
                publishSmoothPlan(smooth_plan_);
            }
            if(BsplineOptimizer_)
            {
                std::cout<<"BsplineOptimizer_ is enabled"<<std::endl;
                //Bspline Optimization
                std::vector<geometry_msgs::PoseStamped> plan_opt;
                safety_corridor_->trajectoryOptimize(plan, corridors_, plan_opt);
                
                // add orientations if needed
                orientation_filter_->processPath(start, plan_opt);

                //publish the plan for visualization purposes
                publishSmoothPlan(plan_opt);
                plan = plan_opt;
            }
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    

    delete potential_array_;
    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void GlobalPlanner::publishSmoothPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    smooth_plan_pub_.publish(gui_path);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

std::vector<geometry_msgs::PoseStamped> GlobalPlanner::simpleCurvatureOptimization(
    const std::vector<geometry_msgs::PoseStamped>& original_path,
    double curvature_threshold) {
    
    if (original_path.size() <= 3) return original_path;
    
    std::vector<geometry_msgs::PoseStamped> optimized_path = original_path;
    
    // 1. 计算原始路径曲率
    std::vector<double> original_curvatures = calculateAllCurvatures(optimized_path);
    
    // 2. 寻找第一个高曲率点（避障开始点）
    int avoidance_start = findFirstHighCurvaturePoint(original_curvatures, curvature_threshold);
    
    // 3. 如果没有找到高曲率点，直接返回
    if (avoidance_start == -1) {
        
        return optimized_path;
    }
    
    // 4. 将避障开始点向前移动
    int moved_index = moveAvoidanceStartForward(optimized_path, avoidance_start);
    
    // 5. 检查移动后路径点的间距，如果过大则进行插值
    if (moved_index < avoidance_start) {
        optimized_path = interpolatePathPoints(optimized_path, moved_index, avoidance_start);
    }
    
    // 6. 验证曲率是否改善
    std::vector<double> new_curvatures = calculateAllCurvatures(optimized_path);
    double original_max_curvature = 0.0;
    double new_max_curvature = 0.0;
    
    // 计算避障区域的曲率变化
    for (int i = std::max(1, avoidance_start-2); i < std::min((int)new_curvatures.size()-1, avoidance_start+3); i++) {
        original_max_curvature = std::max(original_max_curvature, original_curvatures[i]);
        new_max_curvature = std::max(new_max_curvature, new_curvatures[i]);
    }
    
    ROS_INFO("Curvature check: original=%.3f, new=%.3f, improvement=%.1f%%", 
              original_max_curvature, new_max_curvature,
              (original_max_curvature - new_max_curvature) / original_max_curvature * 100);
    
    // 7. 如果曲率没有改善，回退到原始路径
    if (new_max_curvature >= original_max_curvature * 0.9) {
        ROS_INFO("Curvature not improved, reverting to original path");
        return original_path;
    }
    
    return optimized_path;

}
int GlobalPlanner::moveAvoidanceStartForward(std::vector<geometry_msgs::PoseStamped>& path,
                                            int avoidance_start_index) {
    
    int best_candidate = avoidance_start_index;
    
    // 尝试不同的移动距离，找到最优的
    for (int move_dist = 1; move_dist <= 3; move_dist++) {
        int candidate_index = avoidance_start_index - move_dist;
        
        // 检查边界
        if (candidate_index < 1) break;
        
        // 检查候选点是否安全
        if (isPointSafe(path[candidate_index])) {
            // 用候选点替换避障开始点
            path[avoidance_start_index] = path[candidate_index];
            best_candidate = candidate_index;
            
            ROS_DEBUG("Moved avoidance start from index %d to %d", 
                     avoidance_start_index, candidate_index);
            
            // 如果移动距离较大，提前返回以便插值
            if (move_dist >= 2) {
                break;
            }
        }
    }
    
    return best_candidate;
}


std::vector<double> GlobalPlanner::calculateAllCurvatures(const std::vector<geometry_msgs::PoseStamped>& path) {
    std::vector<double> curvatures(path.size(), 0.0);
    
    for (int i = 1; i < path.size() - 1; i++) {
        const auto& p1 = path[i-1].pose.position;
        const auto& p2 = path[i].pose.position;
        const auto& p3 = path[i+1].pose.position;
        
        curvatures[i] = calculateThreePointCurvature(p1, p2, p3);
    }
    
    return curvatures;
}
std::vector<geometry_msgs::PoseStamped> GlobalPlanner::interpolatePathPoints(
    const std::vector<geometry_msgs::PoseStamped>& path,
    int start_index, int end_index) {
    
    if (end_index <= start_index + 1) {
        return path; // 不需要插值
    }
    
    std::vector<geometry_msgs::PoseStamped> interpolated_path = path;
    
    // 计算需要插入的点数
    int points_to_insert = end_index - start_index - 1;
    
    if (points_to_insert <= 0) {
        return path;
    }
    
    const auto& start_point = path[start_index].pose.position;
    const auto& end_point = path[end_index].pose.position;
    
    // 在移动点和原始点之间插入平滑过渡点
    for (int i = 1; i <= points_to_insert; i++) {
        double ratio = static_cast<double>(i) / (points_to_insert + 1);
        
        // 创建新的插值点
        geometry_msgs::PoseStamped new_point;
        new_point.header = path[start_index].header;
        
        // 线性插值位置
        new_point.pose.position.x = start_point.x + ratio * (end_point.x - start_point.x);
        new_point.pose.position.y = start_point.y + ratio * (end_point.y - start_point.y);
        new_point.pose.position.z = 0.0;
        
        // 插值方向（可选，可以计算两点之间的方向）
        // double yaw = atan2(end_point.y - start_point.y, end_point.x - start_point.x);
        // new_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        
        // 在路径中插入新点
        interpolated_path.insert(interpolated_path.begin() + start_index + i, new_point);
    }
    
    ROS_DEBUG("Inserted %d points between indices %d and %d", 
             points_to_insert, start_index, end_index);
    
    return interpolated_path;
}
double GlobalPlanner::calculateThreePointCurvature(const geometry_msgs::Point& p1,
                                   const geometry_msgs::Point& p2,
                                   const geometry_msgs::Point& p3) {
    
    double dx1 = p2.x - p1.x, dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x, dy2 = p3.y - p2.y;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double dot_product = dx1 * dx2 + dy1 * dy2;
    
    double mag1 = sqrt(dx1*dx1 + dy1*dy1);
    double mag2 = sqrt(dx2*dx2 + dy2*dy2);
    
    if (mag1 < 1e-10 || mag2 < 1e-10) {
        return 0.0;
    }
    
    // 使用叉积与模长的比值作为曲率度量
    return fabs(cross_product) / (mag1 * mag2);
}
// double GlobalPlanner::calculateDistance(const geometry_msgs::Point& p1, 
//                                        const geometry_msgs::Point& p2) {
//     return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
// }

// double GlobalPlanner::calculatePathSegmentLength(const std::vector<geometry_msgs::PoseStamped>& path,
//                                                int start_index, int end_index) {
//     double length = 0.0;
//     for (int i = start_index; i < end_index; i++) {
//         length += calculateDistance(path[i].pose.position, path[i+1].pose.position);
//     }
//     return length;
// }
bool GlobalPlanner::isPointSafe(const geometry_msgs::PoseStamped& pose) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        return false; // 点在地图外
    }
    
    unsigned char cost = costmap_->getCost(mx, my);
    
    // 检查是否为自由空间或低代价区域
    return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool GlobalPlanner::isPathSegmentSafe(const std::vector<geometry_msgs::PoseStamped>& path,
                                     int start_index, int end_index) {
    for (int i = start_index; i <= end_index; i++) {
        if (!isPointSafe(path[i])) {
            return false;
        }
    }
    return true;
}
int GlobalPlanner::findFirstHighCurvaturePoint(const std::vector<double>& curvatures,
                                              double curvature_threshold) {
    
    // 寻找第一个超过阈值的高曲率点
    for (int i = 1; i < curvatures.size() - 1; i++) {
        if (curvatures[i] > curvature_threshold) {
            return i;
        }
    }
    
    return -1; // 没有找到高曲率点
}
} //end namespace global_planner
