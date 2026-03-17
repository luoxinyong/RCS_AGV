find_package(OpenCV REQUIRED QUIET)

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARY_DIRS})