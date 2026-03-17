/*********************************************************************
 safety corridor.cpp
 Author:luoxinyong
 data: 2025.11.5
 *********************************************************************/
#include <global_planner/safety_corridor.h>

#include <ros/ros.h>
#include <numeric>
#include <algorithm>

namespace global_planner {

    
SafetyCorridor::SafetyCorridor(costmap_2d::Costmap2D* costmap_ros, 
                             double safety_range, int max_iteration)
    : costmap_(costmap_ros)
    , safety_range_(safety_range)
    , max_iteration_(max_iteration) {
    resolution_ = costmap_->getResolution();
}


bool SafetyCorridor::generateCorridors(const std::vector<geometry_msgs::PoseStamped>& path,
                                      const std::string& frame_id,
                                      std::vector<geometry_msgs::PolygonStamped>& corridors,
                                      std::vector<geometry_msgs::PoseStamped>& pruned_waypoints) {
    if (path.empty()) {
        ROS_WARN("Empty path provided for safety corridor generation");
        return false;
    }

    corridors.clear();
    pruned_waypoints.clear();

    int step = 1;
    int safety_range_in_pixel = static_cast<int>(safety_range_ / resolution_);
    int nums = static_cast<int>(path.size());

    for (int i = 0; i < nums - 1; i++) {
        const auto& pose = path[i].pose.position;
        Point2D wp(pose.x, pose.y);

        // Convert world coordinates to map coordinates
        unsigned int mx, my;
        if (!costmap_->worldToMap(wp.x, wp.y, mx, my)) {
            ROS_WARN("Waypoint %d is outside costmap, skipping corridor generation", i);
            continue;
        }
        int x = static_cast<int>(mx), y = static_cast<int>(my);

        // Adjustment for initial position
        int init_step = static_cast<int>(INITIAL_RADIUS / resolution_);
        if (isRectCollision(x - init_step, y - init_step, x + init_step, y + init_step)) {
            int inc = 4;
            int real_x = x, real_y = y;
            bool found_safe_position = false;

            while (inc < max_iteration_) {
                int it = inc / 4;
                int edge = inc % 4;
                inc += 1;

                real_x = x;
                real_y = y;

                if (edge == 0) {
                    real_x = x - it * step;
                } else if (edge == 1) {
                    real_y = y - it * step;
                } else if (edge == 2) {
                    real_x = x + it * step;
                } else {
                    real_y = y + it * step;
                }

                // Check bounds
                if (real_x < 0 || real_x >= costmap_->getSizeInCellsX() ||
                    real_y < 0 || real_y >= costmap_->getSizeInCellsY()) {
                    continue;
                }

                // Collision detection
                if (!isRectCollision(real_x - init_step, real_y - init_step, 
                                   real_x + init_step + 1, real_y + init_step + 1)) {
                    found_safe_position = true;
                    break;
                }
            }

            if (!found_safe_position) {
                ROS_WARN("Failed to find safe initial position for waypoint %d", i);
                return false;
            } else {
                x = real_x;
                y = real_y;
                // Convert back to world coordinates
                double wx, wy;
                costmap_->mapToWorld(x, y, wx, wy);
                wp.x = wx;
                wp.y = wy;
            }
        }

        // Main loop for corridor construction
        int inc = 4;
        std::vector<int> block(4, 0), increment(4, 0);
        
        while (inc < max_iteration_ && (std::accumulate(block.begin(), block.end(), 0) != 4)) {
            int it = inc / 4;
            int edge = inc % 4;
            inc += 1;

            if (block[edge]) {
                continue;
            }

            increment[edge] = it * step;

            if (increment[edge] >= safety_range_in_pixel || isEdgeCollision(edge, x, y, increment)) {
                block[edge] = 1;
            }
        }

        if (inc >= max_iteration_) {
            ROS_WARN("Max iteration reached for corridor construction at waypoint %d", i);
            return false;
        } else {
            // Save pruned waypoint
            geometry_msgs::PoseStamped pruned_pose;
            pruned_pose.header = path[i].header;
            pruned_pose.pose.position.x = wp.x;
            pruned_pose.pose.position.y = wp.y;
            pruned_pose.pose.orientation = path[i].pose.orientation;
            pruned_waypoints.push_back(pruned_pose);

            // Create corridor polygon
            Polygon2D polygon;
            polygon.points = {
                {wp.x + increment[2] * resolution_, wp.y + increment[3] * resolution_},
                {wp.x - increment[0] * resolution_, wp.y + increment[3] * resolution_},
                {wp.x - increment[0] * resolution_, wp.y - increment[1] * resolution_},
                {wp.x + increment[2] * resolution_, wp.y - increment[1] * resolution_}
            };

            geometry_msgs::PolygonStamped corridor_poly;
            polygon.toPolygonStamped(corridor_poly, frame_id);
            corridors.push_back(corridor_poly);
        }
    }

    ROS_INFO("Generated %zu safety corridors for path with %zu waypoints", 
             corridors.size(), path.size());
    return true;
}

    bool SafetyCorridor::trajectoryOptimize(
        const std::vector<geometry_msgs::PoseStamped> &prune_waypoints,
        const std::vector<geometry_msgs::PolygonStamped> &corridors,
        std::vector<geometry_msgs::PoseStamped> &optimized_path)
    {
        optimized_path.clear();
        if (prune_waypoints.size() < 3 || prune_waypoints.size() != corridors.size())
        {
            ROS_ERROR("Invalid input for trajectory optimization");
            return false;
        }
        const int cost_code = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::CORRIDOR 
                            | BsplineOptimizer::WAYPOINTS| BsplineOptimizer::OBSTACLE;
        Eigen::MatrixXd waypoints = posesToMatrix(prune_waypoints);
        Eigen::MatrixXd ctrl_pts = optimizer_.BsplineOptimizeTraj(waypoints.transpose(),waypoints.transpose(), corridors, costmap_, 0.2,
                                                                 cost_code,500, 1);

        optimized_path = matrixToPoses(ctrl_pts.transpose());
        return true;
    }


bool SafetyCorridor::isRectCollision(int lx, int ly, int ux, int uy) const {
    // Clamp coordinates to map bounds
    int map_width = costmap_->getSizeInCellsX();
    int map_height = costmap_->getSizeInCellsY();
    
    lx = std::max(0, lx);
    ly = std::max(0, ly);
    ux = std::min(map_width - 1, ux);
    uy = std::min(map_height - 1, uy);

    for (int x = lx; x <= ux; x++) {
        for (int y = ly; y <= uy; y++) {
            Point2D pt(x, y);
            if (isObstacleInMap(pt)) {
                return true;
            }
        }
    }
    return false;
}

bool SafetyCorridor::isEdgeCollision(int edge, int x, int y, const std::vector<int>& increment) const {
    if (edge == 0 || edge == 2) {  // Left or right edge
        int new_x = (edge == 0) ? x - increment[edge] : x + increment[edge];
        for (int iy = y - increment[1]; iy <= y + increment[3]; iy++) {
            Point2D pt(new_x, iy);
            if (isObstacleInMap(pt)) {
                return true;
            }
        }
        return false;
    } else if (edge == 1 || edge == 3) {  // Bottom or top edge
        int new_y = (edge == 1) ? y - increment[edge] : y + increment[edge];
        for (int ix = x - increment[0]; ix <= x + increment[2]; ix++) {
            Point2D pt(ix, new_y);
            if (isObstacleInMap(pt)) {
                return true;
            }
        }
        return false;
    } else {
        ROS_ERROR("Edge index must be 0, 1, 2, 3. Got: %d", edge);
        return false;
    }
}

bool SafetyCorridor::isObstacleInMap(const Point2D& pt) const {
    unsigned int mx = static_cast<unsigned int>(pt.x);
    unsigned int my = static_cast<unsigned int>(pt.y);
    
    if (mx >= costmap_->getSizeInCellsX() || 
        my >= costmap_->getSizeInCellsY()) {
        return true;  // Consider out-of-bounds as obstacle
    }

    unsigned char cost = costmap_->getCost(mx, my);
    return (cost == costmap_2d::LETHAL_OBSTACLE || 
            cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

// Polygon2D member functions
bool SafetyCorridor::Polygon2D::isPointInside(const Point2D& pt) const {
    if (points.size() < 3) return false;

    bool inside = false;
    for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
        if (((points[i].y > pt.y) != (points[j].y > pt.y)) &&
            (pt.x < (points[j].x - points[i].x) * (pt.y - points[i].y) / 
                    (points[j].y - points[i].y) + points[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

void SafetyCorridor::Polygon2D::toPolygonStamped(geometry_msgs::PolygonStamped& polygon, 
                                                const std::string& frame_id) const {
    polygon.header.frame_id = frame_id;
    polygon.header.stamp = ros::Time::now();
    polygon.polygon.points.resize(points.size());

    for (size_t i = 0; i < points.size(); i++) {
        polygon.polygon.points[i].x = points[i].x;
        polygon.polygon.points[i].y = points[i].y;
        polygon.polygon.points[i].z = 0.0;
    }
}
    Eigen::MatrixXd SafetyCorridor::posesToMatrix(const std::vector<geometry_msgs::PoseStamped> &poses)
    {
        int num_poses = poses.size();
        Eigen::MatrixXd matrix(3, num_poses);

        for (int i = 0; i < num_poses; ++i)
        {
            matrix(0, i) = poses[i].pose.position.x;
            matrix(1, i) = poses[i].pose.position.y;
            matrix(2, i) = poses[i].pose.position.z;
        }

        return matrix;
    }

    std::vector<geometry_msgs::PoseStamped> SafetyCorridor::matrixToPoses(const Eigen::MatrixXd &matrix)
    {
        std::vector<geometry_msgs::PoseStamped> poses;

        if (matrix.rows() == 3)
        { // 3×N矩阵
            for (int i = 0; i < matrix.cols(); ++i)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = matrix(0, i);
                pose.pose.position.y = matrix(1, i);
                pose.pose.position.z = matrix(2, i);
                pose.pose.orientation.w = 1.0; // 默认方向
                poses.push_back(pose);
            }
        }

        return poses;
    }

} // namespace global_planner