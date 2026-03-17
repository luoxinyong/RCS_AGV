#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// #include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/imgproc.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace h_x {
    struct SubmapOrigin {
        float x;
        float y;
        float z;
        int index;
        SubmapOrigin() : x(0), y(0), z(0), index(0) {}
    };

    template<typename T>
    class SubmapHeap:public std::vector<T> {
    public:
        template<typename Func_T>
        SubmapHeap(Func_T cmp):cmp_(cmp){}
        void push(const T&a) {
            this->push_back(a);
            std::push_heap(this->begin(),this->end(),cmp_);
            return;
        }
        void pop() {
            std::pop_heap(this->begin(),this->end(),cmp_);
            this->pop_back();
            return;
        }
        T &top() {
            return this->at(0);
        }
    private:
        std::function<bool(T,T)> cmp_;
    };

}
#endif
