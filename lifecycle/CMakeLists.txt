cmake_minimum_required(VERSION 3.5)

project(lifecycle)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

### demos
add_executable(lifecycle_talker
  src/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
  "lifecycle_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
)
add_executable(lifecycle_listener
  src/lifecycle_listener.cpp)
ament_target_dependencies(lifecycle_listener
  "lifecycle_msgs"
  "rclcpp"
  "std_msgs"
)
add_executable(lifecycle_service_client
  src/lifecycle_service_client.cpp)
ament_target_dependencies(lifecycle_service_client
  "lifecycle_msgs"
  "rclcpp"
)

install(TARGETS
  lifecycle_talker
  lifecycle_listener
  lifecycle_service_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ros_testing REQUIRED)
  add_ros_test(
    test/test_lifecycle.py
    TIMEOUT 60
  )
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
