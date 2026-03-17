#ifndef SLAM_AD_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define SLAM_AD_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace h_x {
class NoFilter: public CloudFilterInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif