cmake_minimum_required(VERSION 3.5)

project(logging_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(composition REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfigLogger.srv"
)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
#set(node_plugins "")

add_library(logger_config_component SHARED
  src/logger_config_component.cpp)
target_compile_definitions(logger_config_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(logger_config_component
  "class_loader"
  "rclcpp")
rosidl_target_interfaces(logger_config_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_register_node_plugins(logger_config_component "logging_demo::LoggerConfig")
#set(node_plugins "${node_plugins}composition::Server;$<TARGET_FILE:logger_config_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  logger_config_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
