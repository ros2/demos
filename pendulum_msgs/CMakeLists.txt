cmake_minimum_required(VERSION 3.5)

project(pendulum_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(pendulum_msgs
  "msg/JointState.msg"
  "msg/JointCommand.msg"
  "msg/RttestResults.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
