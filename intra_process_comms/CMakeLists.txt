cmake_minimum_required(VERSION 2.8.3)

project(intra_process_comms_example)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

ament_package()

##
## Example of intra-process comms
##

add_executable_for_each_rmw_implementations(two_node_pipeline
  src/two_node_pipeline/two_node_pipeline.cpp
  TARGET_DEPENDENCIES "rclcpp" "std_msgs"
  INSTALL)

add_executable_for_each_rmw_implementations(cyclic_pipeline
  src/cyclic_pipeline/cyclic_pipeline.cpp
  TARGET_DEPENDENCIES "rclcpp" "std_msgs"
  INSTALL)

add_executable_for_each_rmw_implementations(image_pipeline_all_in_one
  src/image_pipeline/image_pipeline_all_in_one.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "std_msgs" "OpenCV"
  INSTALL)
