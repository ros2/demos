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
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

function(qos_demo_executable target)
  add_executable(${target}
    src/${target}.cpp
    src/common_nodes.cpp
    src/utils.cpp)
  target_link_libraries(${target}
    rclcpp::rclcpp
    rcutils::rcutils
    ${std_msgs_TARGETS})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

qos_demo_executable(liveliness)
qos_demo_executable(lifespan)
qos_demo_executable(deadline)
qos_demo_executable(interactive_publisher)
qos_demo_executable(interactive_subscriber)
qos_demo_executable(incompatible_qos)

add_library(message_lost SHARED
  src/message_lost_listener.cpp
  src/message_lost_talker.cpp)
target_link_libraries(message_lost
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${sensor_msgs_TARGETS})
target_compile_definitions(message_lost
  PRIVATE "QUALITY_OF_SERVICE_DEMO_BUILDING_DLL")
rclcpp_components_register_node(message_lost
  PLUGIN "quality_of_service_demo::MessageLostListener"
  EXECUTABLE message_lost_listener)
rclcpp_components_register_node(message_lost
  PLUGIN "quality_of_service_demo::MessageLostTalker"
  EXECUTABLE message_lost_talker)

add_library(qos_overrides SHARED
  src/qos_overrides_listener.cpp
  src/qos_overrides_talker.cpp
)
target_link_libraries(qos_overrides
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${sensor_msgs_TARGETS})
target_compile_definitions(qos_overrides
  PRIVATE "QUALITY_OF_SERVICE_DEMO_BUILDING_DLL")
rclcpp_components_register_node(qos_overrides
  PLUGIN "quality_of_service_demo::QosOverridesListener"
  EXECUTABLE qos_overrides_listener)
rclcpp_components_register_node(qos_overrides
  PLUGIN "quality_of_service_demo::QosOverridesTalker"
  EXECUTABLE qos_overrides_talker)

install(TARGETS
  message_lost qos_overrides
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY params_file DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
