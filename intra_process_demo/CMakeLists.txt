cmake_minimum_required(VERSION 2.8.3)

project(intra_process_demo)

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
## Demos
##

# Simple example of using unique_ptr to minimize copies.
add_executable_for_each_rmw_implementations(two_node_pipeline
  src/two_node_pipeline/two_node_pipeline.cpp
  TARGET_DEPENDENCIES "rclcpp" "std_msgs"
  INSTALL)

# Simple example of a cyclic pipeline which uses no allocation while iterating.
add_executable_for_each_rmw_implementations(cyclic_pipeline
  src/cyclic_pipeline/cyclic_pipeline.cpp
  TARGET_DEPENDENCIES "rclcpp" "std_msgs"
  INSTALL)

# A single program with one of each of the image pipeline demo nodes.
add_executable_for_each_rmw_implementations(image_pipeline_all_in_one
  src/image_pipeline/image_pipeline_all_in_one.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "OpenCV"
  INSTALL)

# A single program with one of each of the image pipeline demo nodes, but two image views.
add_executable_for_each_rmw_implementations(image_pipeline_all_in_one_with_two_image_view
  src/image_pipeline/image_pipeline_all_in_one_with_two_image_view.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "OpenCV"
  INSTALL)

# A stand alone node which produces images from a camera using OpenCV.
add_executable_for_each_rmw_implementations(camera_node
  src/image_pipeline/camera_node.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "OpenCV"
  INSTALL)

# A stand alone node which adds some text to an image using OpenCV before passing it along.
add_executable_for_each_rmw_implementations(watermark_node
  src/image_pipeline/watermark_node.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "OpenCV"
  INSTALL)

# A stand alone node which consumes images and displays them using OpenCV.
add_executable_for_each_rmw_implementations(image_view_node
  src/image_pipeline/image_view_node.cpp
  TARGET_DEPENDENCIES "rclcpp" "sensor_msgs" "OpenCV"
  INSTALL)
