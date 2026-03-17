#include "sensor_data/imu_data.hpp"
#include <ros/ros.h>
#include <cmath>
#include "glog/logging.h"

namespace h_x {
    Eigen::Matrix3f IMUData::GetOrientationMatrix() const {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);

        /******** ydd add， 199号车的包中的 imu 方向刚好跟之前 198号车中 imu 方向 差 180度 *******/  
        // TODO: need find the different between 198 and 199 car imu
        // Eigen::Vector3d angular_delta = Eigen::Vector3d(0,0,3.1415926535897931);
        // double angular_delta_mag = angular_delta.norm();
        // // direction:
        // Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

        // // build delta q: // use 罗德里戈公式 使用角轴的方式计算 等同的四元素乘法，四元素乘法等同于按照对应角轴的旋转
        // double angular_delta_cos = cos(angular_delta_mag / 2.0);  // theta/2
        // double angular_delta_sin = sin(angular_delta_mag / 2.0);  // theta/2
        // Eigen::Quaterniond dq(angular_delta_cos,
        //                         angular_delta_sin * angular_delta_dir.x(),
        //                         angular_delta_sin * angular_delta_dir.y(),
        //                         angular_delta_sin * angular_delta_dir.z());

        // // update 
        // q = q * dq;
        return q.matrix().cast<float>();
    }


    bool IMUData::SyncData(std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& UnsyncedData, std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& SyncedData, const double& sync_time) {
        // ROS_INFO(" IMUData::SyncData enter ...... ");
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time) {
                double dt =  UnsyncedData.front().time - sync_time;
                ROS_ERROR(" IMUData_err_1:  [%f, %f, %f]", UnsyncedData.front().time, sync_time, dt);
                return false;
            }
            //在这里实现了整个滑动窗口式的数据筛选，每次只丢掉队首的数据，然后比较后一个点是否在激光之后
            if (UnsyncedData.at(1).time < sync_time) {
                // LOG(ERROR) << " dt: " << UnsyncedData.front().time - sync_time << std::endl;
                UnsyncedData.pop_front();
                continue;
            }
            // sync_time - UnsyncedData.front().time should be <= 0.2:
            if (sync_time - UnsyncedData.front().time > 0.2) {
                ROS_ERROR(" IMUData_err_2: front imu less than sync_time 0.2 ");
                UnsyncedData.pop_front();
                return false;
            }
            // UnsyncedData.at(1).time - sync_time should be <= 0.2
            if (UnsyncedData.at(1).time - sync_time > 0.2) {
                ROS_ERROR(" IMUData_err_3: behind imu large than sync_time 0.2 ");
                UnsyncedData.pop_front();
                return false;
            }
            break;
        }
        if (UnsyncedData.size() < 2) {
            // LOG(ERROR) << " IMUData_err_5: UnsyncedData size less than 2, dt: " << UnsyncedData.front().time - sync_time  << std::endl;
            if(UnsyncedData.size()) {
                double dt = UnsyncedData.front().time - sync_time;
                if(std::fabs(dt) > 0.01) {
                    ROS_ERROR(" IMUData_err_4: dt: %f", dt);
                }
                SyncedData.emplace_back(UnsyncedData.at(0));
                return true;
            } else {
                ROS_ERROR(" IMUData_err_5: imu unsynced data is empty ...... ");
            }
            return false;
        }    

        IMUData front_data = UnsyncedData.at(0);
        IMUData back_data = UnsyncedData.at(1);
        IMUData synced_data;
        
        // |← δt_front（back_scale） →|← δt_back（front_scale） →|
        // front_data.time ──── sync_time ──── back_data.time
        // |←────────── Δt ─────────→|
        //线性插值：value = 前点值 × (后段间隔占比) + 后点值 × (前段间隔占比)
        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);//后一段时间占比
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);//前一段时间占比
        synced_data.time = sync_time;
        synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
        synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
        // synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
        synced_data.linear_acceleration.z = 9.80943;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
        // 线性插值之后要归一化
        synced_data.orientation.Normlize();

        SyncedData.emplace_back(synced_data);

        return true;
}




}