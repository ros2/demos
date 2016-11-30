cmake_minimum_required(VERSION 3.5)

project(topic_monitor)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/data_pub.py
  scripts/launch_data_pub.py
  scripts/topic_monitor.py
  DESTINATION bin
)

ament_package()
