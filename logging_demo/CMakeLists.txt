cmake_minimum_required(VERSION 3.5)

project(logging_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in packages generating python
  # messages because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfigLogger.srv"
)

include_directories(include)

add_library(logger_config_component SHARED
  src/logger_config_component.cpp)
target_compile_definitions(logger_config_component
  PRIVATE "LOGGING_DEMO_BUILDING_DLL")
ament_target_dependencies(logger_config_component
  "class_loader"
  "rclcpp")
rosidl_target_interfaces(logger_config_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_register_node_plugins(logger_config_component "logging_demo::LoggerConfig")

add_library(logger_usage_component SHARED
  src/logger_usage_component.cpp)
target_compile_definitions(logger_usage_component
  PRIVATE "LOGGING_DEMO_BUILDING_DLL")
ament_target_dependencies(logger_usage_component
  "class_loader"
  "rclcpp"
  "std_msgs")
rclcpp_register_node_plugins(logger_usage_component "logging_demo::LoggerUsage")

add_executable(logging_demo_main
  src/logging_demo_main.cpp)
target_link_libraries(logging_demo_main
  logger_config_component
  logger_usage_component)
ament_target_dependencies(logging_demo_main
  "rclcpp")

install(TARGETS
  logger_config_component
  logger_usage_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  logging_demo_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  set(generated_python_files)
  macro(tests)
    set(LOGGING_DEMO_MAIN_EXECUTABLE $<TARGET_FILE:logging_demo_main>)
    set(
      EXPECTED_OUTPUT_LOGGING_DEMO_MAIN_DEFAULT_SEVERITY
      "${CMAKE_CURRENT_SOURCE_DIR}/test/logging_demo_main_default_severity")
    set(
      EXPECTED_OUTPUT_LOGGING_DEMO_MAIN_DEBUG_SEVERITY
      "${CMAKE_CURRENT_SOURCE_DIR}/test/logging_demo_main_debug_severity")

    configure_file(
      test/test_logging_demo.py.in
      test_logging_demo${target_suffix}.py.genexp
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_logging_demo${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_logging_demo${target_suffix}.py.genexp"
    )
    ament_add_pytest_test(test_logging_demo${target_suffix}
      "${CMAKE_CURRENT_BINARY_DIR}/test_logging_demo${target_suffix}_$<CONFIG>.py"
      ENV RMW_IMPLEMENTATION=${rmw_implementation}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 30)
    list(
      APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/test_logging_demo${target_suffix}_$<CONFIG>.py")
  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    # the generated code might contain longer lines if from a template
    MAX_LINE_LENGTH 999
    ${generated_python_files})
endif()

ament_package()
