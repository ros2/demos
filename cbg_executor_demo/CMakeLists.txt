cmake_minimum_required(VERSION 3.5)
project(cbg_executor_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(ping_and_pong_node main.cpp src/cbg_executor_demo/PingNode.cpp src/cbg_executor_demo/PongNode.cpp)
ament_target_dependencies(ping_and_pong_node rclcpp std_msgs)

install(TARGETS ping_and_pong_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_package()
