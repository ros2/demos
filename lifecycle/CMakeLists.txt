cmake_minimum_required(VERSION 3.5)

project(lifecycle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(lifecycle_talker
  src/lifecycle_talker.cpp)

ament_auto_add_executable(lifecycle_listener
  src/lifecycle_listener.cpp)

ament_auto_add_executable(lifecycle_service_client
  src/lifecycle_service_client.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/test_lifecycle.py
    TIMEOUT 60
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch
)
