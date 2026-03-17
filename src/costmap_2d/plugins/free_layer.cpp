#include <costmap_2d/free_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::FreeLayerBounds, costmap_2d::Layer)


namespace costmap_2d
{
     void FreeLayerBounds::onInitialize() 
    {
        ros::NodeHandle nh("~/" + name_);
        nh.param("enabled", enabled_, true);

        nh.param("size_x", size_x_, 200.0);  // 全局扩展宽度
        nh.param("size_y", size_y_, 200.0);  // 全局扩展高度
        nh.param("origin_x", origin_x_, -200.0);
        nh.param("origin_y", origin_y_, -200.0);

        ROS_INFO("FreeLayerBounds initialized: size=(%.2f, %.2f) origin=(%.2f, %.2f)",
                 size_x_, size_y_, origin_x_, origin_y_);
    }

    void FreeLayerBounds::updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y,
                    double* max_x, double* max_y) 
  {
    if (!enabled_) return;
    ROS_INFO("FreeLayerBounds updateBounds: robot=(%.2f, %.2f, %.2f)",
             robot_x, robot_y, robot_yaw);
    // 以车体中心为原点扩展地图
    *min_x = robot_x - size_x_/2.0;
    *min_y = robot_y - size_y_/2.0;
    *max_x = robot_x + size_x_/2.0;
    *max_y = robot_y + size_y_/2.0;

    ROS_DEBUG("FreeLayerCenter bounds: min=(%.2f,%.2f) max=(%.2f,%.2f)",
              *min_x, *min_y, *max_x, *max_y);
  }

    void FreeLayerBounds::updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j) 
    {
     
        // for (int i = min_i; i < max_i; ++i)
        // {
        //     for (int j = min_j; j < max_j; ++j)
        //     {
        //         master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
        //     }
        // }
    }
};
