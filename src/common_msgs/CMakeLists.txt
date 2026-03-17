cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  nav_data.msg
  rcs_cmd_data.msg
  ipc_res_data.msg
  plc_res_nav_data.msg
  plc_res_action_data.msg
  one_action_task.msg
  action_data.msg
  mapping_order.msg
  trans.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
	  nav_msgs
)

catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
