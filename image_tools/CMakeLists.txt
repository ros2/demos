cmake_minimum_required(VERSION 2.8.3)

project(image_tools)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

add_executable_for_each_rmw_implementations(cam2image
  src/cam2image.cpp
  src/options.cpp
  TARGET_DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  INSTALL
)
add_executable_for_each_rmw_implementations(showimage
  src/showimage.cpp
  src/options.cpp
  TARGET_DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  INSTALL
)

ament_package()
