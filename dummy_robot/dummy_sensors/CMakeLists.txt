cmake_minimum_required(VERSION 3.5)

project(dummy_sensors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dummy_laser src/dummy_laser.cpp)
ament_target_dependencies(dummy_laser
  "rclcpp"
  "sensor_msgs"
)

add_executable(dummy_joint_states src/dummy_joint_states.cpp)
ament_target_dependencies(dummy_joint_states
  "rclcpp"
  "sensor_msgs")
install(TARGETS
  dummy_joint_states
  dummy_laser
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
