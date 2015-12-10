cmake_minimum_required(VERSION 2.8.3)

project(pendulum_control)

if(APPLE OR WIN32)
  message(STATUS "The pendulum_control demo is only supported on Linux. Package will not be built.")
  return()
endif()

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rttest)

include_directories(include)

if(NOT rttest_FOUND)
  message(STATUS "rttest not found. pendulum_control package will not be built.")
  return()
endif()

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(pendulum_demo${target_suffix}
    src/pendulum_demo.cpp)
  ament_target_dependencies(pendulum_demo${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  add_executable(pendulum_logger${target_suffix}
    src/pendulum_logger.cpp)
  ament_target_dependencies(pendulum_logger${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  add_executable(pendulum_teleop${target_suffix}
    src/pendulum_teleop.cpp)
  ament_target_dependencies(pendulum_teleop${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  install(TARGETS
    pendulum_demo${target_suffix}
    pendulum_logger${target_suffix}
    pendulum_teleop${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

install(
  DIRECTORY scripts/
  DESTINATION bin
)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
