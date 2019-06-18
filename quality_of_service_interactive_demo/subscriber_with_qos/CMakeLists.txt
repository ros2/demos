cmake_minimum_required(VERSION 3.5)
project(subscriber_with_qos_demo)

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

add_executable(subscriber_with_qos subscriber.cpp)
ament_target_dependencies(subscriber_with_qos rclcpp std_msgs)

install(TARGETS
  subscriber_with_qos
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
