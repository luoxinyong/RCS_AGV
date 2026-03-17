set(WORK_SPACE_PATH ${CATKIN_PACKAGE_SHARE_DESTINATION})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/global_defination/global_defination.h.in
  ${PROJECT_BINARY_DIR}/include/global_defination/global_defination.h)
include_directories(${PROJECT_BINARY_DIR}/include)