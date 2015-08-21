cmake_minimum_required(VERSION 2.8.3)

project(intra_process_comms_example)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

##
## Example of intra-process comms
##

add_executable_for_each_rmw_implementations(intrapc
  src/intrapc.cpp
  TARGET_DEPENDENCIES "rclcpp" "std_msgs"
  INSTALL)
