cmake_minimum_required(VERSION 3.20)

project(pendulum_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
