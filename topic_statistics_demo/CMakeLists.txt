cmake_minimum_required(VERSION 3.5)

project(topic_statistics_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(sensor_msgs REQUIRED)
find_package(statistics_msgs REQUIRED)

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
  ${sensor_msgs_TARGETS}
  ${statistics_msgs_TARGETS})
install(TARGETS display_topic_statistics DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
