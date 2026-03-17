#include "sensor_data/pose_data.hpp"

namespace h_x {

    Eigen::Quaternionf PoseData::GetQuaternion() {
        Eigen::Quaternionf q;
        q = pose.block<3,3>(0,0);
        return q;
}
    
}