#include <iostream>

#include <pcl/common/transforms.h>
#include <ros/ros.h>
// #include "glog/logging.h"
#include <yaml-cpp/yaml.h>

#include "Geocentric/LocalCartesian.hpp"
#include "global_defination/global_defination.h"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"

/*
 一楼门口附近初始化数据:
the pose before initializing is:         x: -20.1696 y: 7.43456 z: 0 roll: 0
pitch:0 yaw: -0.0201312 the pose in odom before initializing is: x  -20.1696
y  7.43456 z  0 the icp score in initializing process is: 0.0618466 the pose
after initializing process is: 0.999684 -0.0178443 -0.0179583   -21.2711
 0.0175728   0.999737 -0.0151684    7.44482
 0.0182241  0.0148477   0.999725  -0.304941
         0          0          0          1
transformTobeMapped X_Y_Z: 0 0 0
the pose of odom relative to Map: x:-21.2711 y:7.44482 z:-0.304941
Initializing Succeed
*/

using namespace h_x;

static GeographicLib::LocalCartesian geo_converter;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_gps_to_odom");
  ros::NodeHandle nh;
  // GeographicLib::LocalCartesian h_x::GNSSData::geo_converter;
  /**  移动智地高精车出发原点 gps , -21.2, 7.4 can init success **/
  double longitude_orin = 121.1702651870648282;
  double latitude_orin = 31.1506836219693000;
  double altitude_orin = 4.3672881126403809;

  // double longitude = 121.170180367;
  // double latitude = 31.1508058667;
  // double altitude = 46.53;

  /******   AD3.0 pic
   *   local_E local_N local_U is: [-18.896763, 9.137719,42.162677]
   ******/
  // double longitude = 121.170067013;
  // double latitude = 31.150766038;
  // double altitude = 46.53;

  /******  zhisuo_yiqi_helios.bag
   *   local_E local_N local_U is: [-24.233006, 22.669206,30.762625]
   *    *********/
  // double longitude = 121.17001105;
  // double latitude = 31.150888083333335;
  // double altitude = 35.129999999999995;

  /******  zhisuo_erqi_helios.bag
   *   local_E local_N local_U is: [-22.910735, 16.964774,19.632648]
   *    *********/
  // double longitude = 121.17002486666667;
  // double latitude = 31.1508367;
  // double altitude = 24.03;

  /******   2023-01-20-12-03-52.bag
   *  local_E local_N local_U is: [-28.679732, 18.777610,39.032620]
   * *********/
  // double longitude = 121.16996441666667;
  // double latitude = 31.150852983333333;
  // double altitude = 43.400000000000006;

  /******   2023-01-20-11-14-21.bag
   *   local_E local_N local_U is: [-21.280208, 8.728797,22.502670]
   *    *********/
  // double longitude = 121.17004201666667;
  // double latitude = 31.15076235;
  // double altitude = 26.87;

  /******  2023-01-20-11-17-51.bag
   *    local_E local_N local_U is: [9.199755, -36.357500,16.862601]
   *    *********/
  // double longitude = 121.17036166666666;
  // double latitude = 31.1503557;
  // double altitude = 21.229999999999997;

  /******  2022-08-18-10-14-12.bag
   *    local_E local_N local_U is: [-51.446361, -29.803373, 0.182923]
   *    *********/
  double longitude = 121.16972565742493;
  double latitude = 31.15041481233045;
  double altitude = 4.550487995147705;

  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  geo_converter.Reset(latitude_orin, longitude_orin, altitude_orin);
  geo_converter.Forward(latitude, longitude, altitude, local_E, local_N,
                        local_U);

  ROS_INFO("local_E local_N local_U is: [%f, %f,%f]", local_E, local_N,
           local_U);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}