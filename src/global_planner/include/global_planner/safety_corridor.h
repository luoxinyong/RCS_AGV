/*********************************************************************
 safety_corridor.h
 Author:luoxinyong
 data: 2025.11.5
 *********************************************************************/

#ifndef SAFETY_CORRIDOR_H
#define SAFETY_CORRIDOR_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <nav_msgs/Path.h>

#include "global_planner/bspline_optimizer.h"

namespace global_planner {

/**
 * @brief Safety corridor class for generating collision-free corridors around planned paths
 */
class SafetyCorridor {
public:
    /**
     * @brief Constructor
     * @param costmap_ Pointer to the costmap
     * @param safety_range Safety range for corridor expansion [m]
     * @param max_iteration Maximum iterations for corridor construction
     */
    SafetyCorridor(costmap_2d::Costmap2D* costmap_, double safety_range, int max_iteration);
    
    virtual ~SafetyCorridor() = default;

    /**
     * @brief Generate safety corridors for a given path
     * @param path The planned path
     * @param corridors Output vector of safety corridors as polygons
     * @param pruned_waypoints Output pruned waypoints after redundancy removal
     * @return true if successful construction
     */
    bool generateCorridors(const std::vector<geometry_msgs::PoseStamped>& path, const std::string& frame_id,
                          std::vector<geometry_msgs::PolygonStamped>& corridors,
                          std::vector<geometry_msgs::PoseStamped>& pruned_waypoints);
    bool trajectoryOptimize(
        const std::vector<geometry_msgs::PoseStamped> &prune_waypoints,
        const std::vector<geometry_msgs::PolygonStamped> &corridors,
        std::vector<geometry_msgs::PoseStamped> &optimized_path);

protected:  
    /**
     * @brief 2D point structure
     */
    struct Point2D {
        double x, y;
        Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    };

    /**
     * @brief 2D polygon structure
     */
    struct Polygon2D {
        std::vector<Point2D> points;
        
        bool isPointInside(const Point2D& pt) const;
        void toPolygonStamped(geometry_msgs::PolygonStamped& polygon, const std::string& frame_id) const;
    };

    /**
     * @brief Check if a rectangle collides with obstacles
     * @param lx Lower x coordinate [pixel]
     * @param ly Lower y coordinate [pixel]
     * @param ux Upper x coordinate [pixel]
     * @param uy Upper y coordinate [pixel]
     * @return true if collision occurs
     */
    bool isRectCollision(int lx, int ly, int ux, int uy) const;

    /**
     * @brief Check if a specific edge expansion causes collision
     * @param edge Edge index (0: left, 1: bottom, 2: right, 3: top)
     * @param x Center x coordinate [pixel]
     * @param y Center y coordinate [pixel]
     * @param increment Incremental values for each edge
     * @return true if collision occurs
     */
    bool isEdgeCollision(int edge, int x, int y, const std::vector<int>& increment) const;

    /**
     * @brief Check if a map point is an obstacle
     * @param pt Point in map coordinates
     * @return true if obstacle
     */
    bool isObstacleInMap(const Point2D& pt) const;

    std::vector<geometry_msgs::PoseStamped> matrixToPoses(const Eigen::MatrixXd &matrix);
    Eigen::MatrixXd   posesToMatrix(const std::vector<geometry_msgs::PoseStamped> &poses);

private:
    costmap_2d::Costmap2D* costmap_;
    double safety_range_;
    int max_iteration_;
    double resolution_;
    
    // Radius of initial approved region [m]
    static constexpr double INITIAL_RADIUS = 0.4;

    BsplineOptimizer optimizer_;
};

} // namespace global_planne} //end namespace global_planner
#endif

