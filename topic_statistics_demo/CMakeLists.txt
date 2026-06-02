cmake_minimum_required(VERSION 3.20)

project(topic_statistics_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(display_topic_statistics
  src/imu_talker_listener_nodes.cpp
  src/display_topic_statistics.cpp
  src/string_talker_listener_nodes.cpp
  src/topic_statistics_listener.cpp)
target_link_libraries(display_topic_statistics
  rclcpp::rclcpp
  rcutils::rcutils
  sensor_msgs::sensor_msgs
  statistics_msgs::statistics_msgs
  example_interfaces::example_interfaces
  ament_cmake_ros_core::ament_ros_defaults)
install(TARGETS display_topic_statistics DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
