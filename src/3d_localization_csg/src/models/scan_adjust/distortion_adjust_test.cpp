#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "models/scan_adjust/distortion_adjust.hpp"

namespace h_x
{

    class AdjustCloudTest : public ::testing::Test
    {
    protected:
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr adjusted_cloud_ptr;
        DistortionAdjust distortion_adjust_obj_; // 类实例

        void SetUp() override
        {
            input_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
            adjusted_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());


            // pcl::PointXYZ point;
            // point.x = 1.0;
            // point.y = 1.0;
            // point.z = 0.0;
            // input_cloud_ptr->points.push_back(point);

            // point.x = -1.0;
            // point.y = -1.0;
            // point.z = 0.0;
            // input_cloud_ptr->points.push_back(point);

            
            std::string path_pcd = std::string("/home/ydd/test_file/438.pcd");
            pcl::io::loadPCDFile(path_pcd, *input_cloud_ptr);

        }
    };

    // AdjustRsCloud 测试用例 
    TEST_F(AdjustCloudTest, TestAdjustedRsCloud)
    {
        // 设置其他参数
        float scan_period = 0.1;
        VelocityData velocity_data; // 激光雷达运动速度
        velocity_data.angular_velocity.x = 0;
        velocity_data.angular_velocity.y = 0;
        velocity_data.angular_velocity.z = 0/18.0;

        velocity_data.linear_velocity.x = 0.0;
        velocity_data.linear_velocity.y = 0;
        velocity_data.linear_velocity.z = 0;

        // 调用类的成员函数进行调整
        distortion_adjust_obj_.SetMotionInfo(scan_period, velocity_data);
        distortion_adjust_obj_.AdjustRsCloud(input_cloud_ptr, adjusted_cloud_ptr);

        // 进行验证
        for (size_t i = 0; i < input_cloud_ptr->points.size(); ++i) {
            std::cout << " i: " << i << std::endl;
            EXPECT_NEAR(adjusted_cloud_ptr->points[i].x, input_cloud_ptr->points[i].x, 0.01);
            EXPECT_NEAR(adjusted_cloud_ptr->points[i].y, input_cloud_ptr->points[i].y, 0.01);
            EXPECT_NEAR(adjusted_cloud_ptr->points[i].z, input_cloud_ptr->points[i].z, 0.01);
        }
  
    }

    // AdjustCloud 测试用例
    // TEST_F(AdjustCloudTest, TestAdjustedCloud)
    // {
    //     // 设置其他参数
    //     float scan_period = 0.1;
    //     VelocityData velocity_data; // 激光雷达运动速度
    //     velocity_data.angular_velocity.x = 0;
    //     velocity_data.angular_velocity.y = 0;
    //     velocity_data.angular_velocity.z = 0;

    //     velocity_data.linear_velocity.x = 0;
    //     velocity_data.linear_velocity.y = 0;
    //     velocity_data.linear_velocity.z = 0;

    //     // 调用类的成员函数进行调整
    //     distortion_adjust_obj_.SetMotionInfo(scan_period, velocity_data);
    //     distortion_adjust_obj_.AdjustCloud(input_cloud_ptr, adjusted_cloud_ptr);

    //     // 进行验证
    //     for (size_t i = 0; i < input_cloud_ptr->points.size(); ++i) {
    //         std::cout << " i: " << i << std::endl;
    //         EXPECT_NEAR(adjusted_cloud_ptr->points[0].x, input_cloud_ptr->points[0].x, 0.01);
    //         EXPECT_NEAR(adjusted_cloud_ptr->points[0].y, input_cloud_ptr->points[0].y, 0.01);
    //         EXPECT_NEAR(adjusted_cloud_ptr->points[0].z, input_cloud_ptr->points[0].z, 0.01);
    //     }
       
    // }

    // 主函数，运行所有测试
    int main(int argc, char **argv)
    {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }

}