cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp_native)

find_package(rmw_fastrtps_cpp QUIET)
if(NOT rmw_fastrtps_cpp_FOUND)
  message(STATUS "Could not find rmw_fastrtps_cpp - skipping")
  return()
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(talker_native SHARED
  src/talker.cpp)
target_compile_definitions(talker_native
  PRIVATE "DEMO_NODES_CPP_NATIVE_BUILDING_DLL")
ament_target_dependencies(talker_native
  "rclcpp"
  "std_msgs"
  "rclcpp_components"
  "rmw_fastrtps_cpp")
rclcpp_components_register_node(talker_native PLUGIN "demo_nodes_cpp_native::Talker" EXECUTABLE talker)
install(TARGETS
  talker_native
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(tutorial_executables "talker")

  set(DEMO_NODES_CPP_EXPECTED_OUTPUT "")
  foreach(executable ${tutorial_executables})
    list(APPEND DEMO_NODES_CPP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
  endforeach()

  set(DEMO_NODES_CPP_EXECUTABLE "")
  foreach(executable ${tutorial_executables})
    list(APPEND DEMO_NODES_CPP_EXECUTABLE "$<TARGET_FILE:${executable}>")
  endforeach()

  string(REPLACE ";" "_" exe_list_underscore "${tutorial_executables}")
  configure_file(
    test/test_executables_tutorial.py.in
    test_${exe_list_underscore}.py.configured
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}_$<CONFIG>.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}.py.configured"
  )

  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}_$<CONFIG>.py"
    TARGET test_tutorial_${exe_list_underscore}
    TIMEOUT 30
    ENV
    RCL_ASSERT_RMW_ID_MATCHES=rmw_fastrtps_cpp
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  )
  foreach(executable ${tutorial_executables})
    set_property(
      TEST test_tutorial_${exe_list_underscore}
      APPEND PROPERTY DEPENDS ${executable})
  endforeach()
endif()

ament_package()
