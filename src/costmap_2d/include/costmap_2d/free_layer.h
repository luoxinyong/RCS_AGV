#ifndef COSTMAP_2D_FREE_LAYER_H_
#define COSTMAP_2D_FREE_LAYER_H_

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <ros/ros.h>

namespace costmap_2d {

class FreeLayerBounds : public costmap_2d::Layer
{
public:
    FreeLayerBounds() {}

    virtual void onInitialize() override;
    

    // 核心：每帧都返回超大 bounds，让 master_grid 扩容
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y,
                              double* max_x, double* max_y) override;
   

    // 不写任何代价值，保持障碍物数据不受影响
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j) override;
   



private:
    bool enabled_{true};
    double size_x_{200.0}, size_y_{200.0};
    double origin_x_{-100.0}, origin_y_{-100.0};
};

} // namespace custom_costmap

#endif  