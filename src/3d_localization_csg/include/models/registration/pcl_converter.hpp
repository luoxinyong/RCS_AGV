#ifndef PCL_CONVERTER_H
#define PCL_CONVERTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace h_x {

class PclConverter {
public:
    // 将 PCL PointXYZ 点云转换为 std::vector<Eigen::Vector4f>
    static std::vector<Eigen::Vector4f> toEigenVector4f(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
        std::vector<Eigen::Vector4f> points;
        points.reserve(cloud->size());
        
        for (const auto& p : *cloud) {
            points.emplace_back(p.x, p.y, p.z, 1.0f);  // 第四维设为1.0f
        }
        
        return points;
    }

    // 将 std::vector<Eigen::Vector4f> 转换回 PCL PointXYZ 点云
    static pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(const std::vector<Eigen::Vector4f>& points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(points.size());
        
        for (size_t i = 0; i < points.size(); ++i) {
            auto& p = cloud->points[i];
            p.x = points[i].x();
            p.y = points[i].y();
            p.z = points[i].z();
        }
        
        return cloud;
    }
};

} // namespace h_x

#endif