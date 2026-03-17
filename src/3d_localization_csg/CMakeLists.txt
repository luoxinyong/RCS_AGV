cmake_minimum_required(VERSION 3.0.2)
project(loc_ad)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
  cv_bridge
  roslib
)

find_package(OpenMP) # ndt_omp用到了openMP库
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)
# find_package(GTSAM REQUIRED QUIET)
# find_package(Boost REQUIRED COMPONENTS timer)
find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(small_gicp REQUIRED)

find_library(SMALL_GICP_LIBRARY
  NAMES small_gicp
  PATHS /usr/local/lib /usr/lib /usr/local/lib64 /usr/lib64
  DOC "Path to small_gicp library"
)

if(NOT SMALL_GICP_LIBRARY)
  message(FATAL_ERROR "small_gicp library not found")
else()
  message(STATUS "Found small_gicp: ${SMALL_GICP_LIBRARY}")
endif()
list(APPEND ALL_TARGET_LIBRARIES ${SMALL_GICP_LIBRARY})
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  PosVel.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES loc_ad
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  LIBRARIES small_gicp

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  roslib
)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
  )
endif()

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/small_gicp/include/small_gicp
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/geographic.cmake)
# include(cmake/opencv.cmake)
include(cmake/sophus.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/local/include/small_gicp/*)
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
file(GLOB_RECURSE TEST_SRCS "src/*_test.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${TEST_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

# add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS})
# target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(test_gps_to_odom_node src/test_gps_to_odom_node.cpp ${ALL_SRCS})
# target_link_libraries(test_gps_to_odom_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#GPS imu Republish
add_executable(send_walltime_imu src/send_walltime_imu_node.cpp)
target_link_libraries(send_walltime_imu ${catkin_LIBRARIES})

add_executable(send_walltime_gps src/send_walltime_gps_node.cpp)
target_link_libraries(send_walltime_gps ${catkin_LIBRARIES})

add_executable(send_walltime_odom src/send_walltime_odom_node.cpp)
target_link_libraries(send_walltime_odom ${catkin_LIBRARIES})

# add_executable(republish_vel src/republish_vel_node.cpp)
# target_link_libraries(republish_vel ${catkin_LIBRARIES})


# 添加测试文件
catkin_add_gtest(test_adjusted_cloud
  src/models/scan_adjust/distortion_adjust_test.cpp
  src/models/scan_adjust/distortion_adjust.cpp
)

# 链接测试文件与类库
target_link_libraries(test_adjusted_cloud
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
  GTest::GTest
  GTest::Main
)


add_executable(data_pretreat_node src/localization/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} small_gicp::small_gicp)

add_executable(filtering_node src/localization/filtering_node.cpp ${ALL_SRCS})
add_dependencies(filtering_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(filtering_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} small_gicp::small_gicp)

add_executable(gicp_test src/models/registration/gicp_test.cpp ${ALL_SRCS})
add_dependencies(gicp_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gicp_test ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} small_gicp::small_gicp)

install(TARGETS
        data_pretreat_node
        # vel_use_nowtime_node
        # test_gps_to_odom_node
        filtering_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

install(FILES lib/libsmall_gicp.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    PATTERN "*.xacro"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)
