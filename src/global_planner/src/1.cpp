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
    auto start_time = std::chrono::system_clock::now();

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
         ROS_INFO("use_fot plan");
         //std::cout<<"start.pose.position.x y goal x y world"<<start.pose.position.x<< goal.pose.position.x<<goal.pose.position.y<<std::endl;
         //std::cout<<"start.pose.position.x y goal x y  map"<< start_x<<start_y<<goal_x<<goal_y<<std::endl;
         found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y,
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
          ROS_INFO("get fot path");
            fot_->getpath(plan);
            
            publishPlan(plan);
            publishSafetyCorridors(plan);
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        }
        if(getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
                publishPlan(plan);
                publishSafetyCorridors(plan);
                if (enable_simple_curvature_optimization_ && plan.size() > 3) {
                ROS_INFO("Applying simple curvature optimization");
                plan = simpleCurvatureOptimization(plan, curvature_threshold_);
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
        
        }
        else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    auto end_time = std::chrono::system_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Planner core make plan time: %ld ms", time_used.count());

    delete potential_array_;
    return !plan.empty();
}