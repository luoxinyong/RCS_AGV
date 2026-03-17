#include "tools/common.h"

// sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
// {
//     sensor_msgs::Imu imu_out = imu_in;
//     //
//     这里把imu的数据旋转到前左上坐标系下，可以参考https://github.com/TixiaoShan/LIO-SAM/issues/6
//     // rotate acceleration
//     Eigen::Vector3d acc(imu_in.linear_acceleration.x,
//     imu_in.linear_acceleration.y, imu_in.linear_acceleration.z); acc = extRot
//     * acc; imu_out.linear_acceleration.x = acc.x();
//     imu_out.linear_acceleration.y = acc.y();
//     imu_out.linear_acceleration.z = acc.z();
//     // rotate gyroscope
//     Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
//     imu_in.angular_velocity.z); gyr = extRot * gyr;
//     imu_out.angular_velocity.x = gyr.x();
//     imu_out.angular_velocity.y = gyr.y();
//     imu_out.angular_velocity.z = gyr.z();
//     // rotate roll pitch yaw
//     // 这是一个九轴imu，因此还会有姿态信息
//     Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
//     imu_in.orientation.y, imu_in.orientation.z); Eigen::Quaterniond q_final =
//     q_from * extQRPY; imu_out.orientation.x = q_final.x();
//     imu_out.orientation.y = q_final.y();
//     imu_out.orientation.z = q_final.z();
//     imu_out.orientation.w = q_final.w();
//     // 简单校验一下结果
//     if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() +
//     q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
//     {
//         ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
//         ros::shutdown();
//     }

//     return imu_out;
// }

// sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub,
// pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string
// thisFrame)
// {
//     sensor_msgs::PointCloud2 tempCloud;
//     pcl::toROSMsg(*thisCloud, tempCloud);
//     tempCloud.header.stamp = thisStamp;
//     tempCloud.header.frame_id = thisFrame;
//     if (thisPub->getNumSubscribers() != 0)
//         thisPub->publish(tempCloud);
//     return tempCloud;
// }

template <typename T>
double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}

// template<typename T>
// void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T
// *angular_y, T *angular_z)
// {
//     *angular_x = thisImuMsg->angular_velocity.x;
//     *angular_y = thisImuMsg->angular_velocity.y;
//     *angular_z = thisImuMsg->angular_velocity.z;
// }

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y,
                       T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

// template<typename T>
// void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T
// *rosYaw)
// {
//     double imuRoll, imuPitch, imuYaw;
//     tf::Quaternion orientation;
//     tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
//     tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

//     *rosRoll = imuRoll;
//     *rosPitch = imuPitch;
//     *rosYaw = imuYaw;
// }

// float pointDistance(PointType p)
// {
//     return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
// }

// float pointDistance(PointType p1, PointType p2)
// {
//     return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +
//     (p1.z-p2.z)*(p1.z-p2.z));
// }
