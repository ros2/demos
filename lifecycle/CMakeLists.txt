cmake_minimum_required(VERSION 3.5)

project(lifecycle)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

macro(targets)
  ### demos
  add_executable(lifecycle_talker${target_suffix}
    src/lifecycle_talker.cpp)
  target_link_libraries(lifecycle_talker${target_suffix}
    ${rclcpp_lifecycle_LIBRARIES}
    ${std_msgs_LIBRARIES}
  )
  add_executable(lifecycle_listener${target_suffix}
    src/lifecycle_listener.cpp)
  target_link_libraries(lifecycle_listener${target_suffix}
    ${rclcpp_lifecycle_LIBRARIES}
    ${std_msgs_LIBRARIES}
  )
  add_executable(lifecycle_service_client${target_suffix}
    src/lifecycle_service_client.cpp)
  target_link_libraries(lifecycle_service_client${target_suffix}
    ${rclcpp_lifecycle_LIBRARIES}
    ${std_msgs_LIBRARIES}
  )

  install(TARGETS
    lifecycle_talker${target_suffix}
    lifecycle_listener${target_suffix}
    lifecycle_service_client${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install python demo
install(
  FILES
  src/lifecycle_service_client_py.py
  launch/lifecycle_demo_launch.py
  DESTINATION bin)

ament_package()
