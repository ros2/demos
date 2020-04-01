cmake_minimum_required(VERSION 3.5)

project(quality_of_service_demo_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

function(qos_demo_executable target)
  add_executable(${target}
    src/${target}.cpp
    src/common_nodes.cpp
    src/utils.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "std_msgs"
  )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

qos_demo_executable(liveliness)
qos_demo_executable(lifespan)
qos_demo_executable(deadline)
qos_demo_executable(interactive_publisher)
qos_demo_executable(interactive_subscriber)
qos_demo_executable(incompatible_qos)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
