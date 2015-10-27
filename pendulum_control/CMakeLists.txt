cmake_minimum_required(VERSION 2.8.3)

project(pendulum_control)

if(APPLE OR WIN32)
  message(STATUS "The pendulum_control demo is only supported on Linux. Package will not be built.")
  return()
endif()

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rttest)

include_directories(include)

if(NOT rttest_FOUND)
  message(STATUS "rttest not found. pendulum_control package will not be built.")
  return()
endif()

add_executable_for_each_rmw_implementations(pendulum_demo
  src/pendulum_demo.cpp
  TARGET_DEPENDENCIES
  "rclcpp"
  "pendulum_msgs"
  "rttest"
  INSTALL
)

add_executable_for_each_rmw_implementations(pendulum_logger
  src/pendulum_logger.cpp
  TARGET_DEPENDENCIES
  "rclcpp"
  "pendulum_msgs"
  "rttest"
  INSTALL
)

add_executable_for_each_rmw_implementations(pendulum_teleop
  src/pendulum_teleop.cpp
  TARGET_DEPENDENCIES
  "rclcpp"
  "pendulum_msgs"
  "rttest"
  INSTALL
)

install(
  DIRECTORY scripts/
  DESTINATION bin
)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
