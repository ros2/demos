cmake_minimum_required(VERSION 3.20)

project(dummy_sensors)

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
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dummy_laser src/dummy_laser.cpp)
target_link_libraries(dummy_laser PRIVATE
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

add_executable(dummy_joint_states src/dummy_joint_states.cpp)
target_link_libraries(dummy_joint_states PRIVATE
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)
install(TARGETS
  dummy_joint_states
  dummy_laser
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
