cmake_minimum_required(VERSION 2.8.3)

project(cam2image)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(sensor_interfaces REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} src/cam2image.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rmw_implementation sensor_interfaces OpenCV)

ament_package()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
