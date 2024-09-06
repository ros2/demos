cmake_minimum_required(VERSION 3.5)
project(action_tutorials_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(action_tutorials SHARED
  src/fibonacci_action_client.cpp
  src/fibonacci_action_server.cpp)
rclcpp_components_register_node(action_tutorials PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
rclcpp_components_register_node(action_tutorials PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
target_compile_definitions(action_tutorials
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_include_directories(action_tutorials PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(action_tutorials PRIVATE
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
)

install(TARGETS action_tutorials
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
