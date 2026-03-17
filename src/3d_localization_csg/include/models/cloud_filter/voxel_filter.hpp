#ifndef SLAM_AD_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define SLAM_AD_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace h_x {
class VoxelFilter: public CloudFilterInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(const float& leaf_size_x, const float& leaf_size_y, const float& leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(const float& leaf_size_x, const float& leaf_size_y, const float& leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}
#endif