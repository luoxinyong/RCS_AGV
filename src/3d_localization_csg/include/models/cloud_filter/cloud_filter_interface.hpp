#ifndef SLAM_AD_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define SLAM_AD_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.hpp"

namespace h_x {
class CloudFilterInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif