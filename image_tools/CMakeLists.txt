cmake_minimum_required(VERSION 3.5)

project(image_tools)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(cam2image${target_suffix}
    src/cam2image.cpp
    src/options.cpp)
  ament_target_dependencies(cam2image${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "OpenCV")

  add_executable(showimage${target_suffix}
    src/showimage.cpp
    src/options.cpp)
  ament_target_dependencies(showimage${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "OpenCV")

  install(TARGETS
    cam2image${target_suffix}
    showimage${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
