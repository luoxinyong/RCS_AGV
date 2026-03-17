cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs
  nav_msgs
  common_msgs
  pluginlib
  nav_core
  mpc_path_follower
  tf2_ros
  rostest
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

catkin_package(
  CATKIN_DEPENDS nav_core pluginlib mpc_path_follower
)
set(GREADER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/task_manager.cpp src/m4_socket_client.cpp src/pallet_identify_m4.cpp src/tcp_client.cpp src/rfid_reader.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GREADER_LIB_DIR}/libGReader.so
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH ${GREADER_LIB_DIR}
  INSTALL_RPATH ${GREADER_LIB_DIR}
)

add_executable(${PROJECT_NAME}_node src/task_manager_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 单元测试
catkin_add_gtest(test_task_manager_node src/test_task_manager.cpp)
target_link_libraries(test_task_manager_node ${PROJECT_NAME} ${catkin_LIBRARIES})
 
install(DIRECTORY launch config param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
