cmake_minimum_required(VERSION 3.5)

project(intra_process_demo)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)
ament_export_include_directories(include)

##
## Demos
##

# Simple example of using unique_ptr to minimize copies.
add_executable(two_node_pipeline
  src/two_node_pipeline/two_node_pipeline.cpp)
ament_target_dependencies(two_node_pipeline
  "rclcpp"
  "std_msgs")

  # Simple example of a cyclic pipeline which uses no allocation while iterating.
add_executable(cyclic_pipeline
  src/cyclic_pipeline/cyclic_pipeline.cpp)
ament_target_dependencies(cyclic_pipeline
  "rclcpp"
  "std_msgs")

# A single program with one of each of the image pipeline demo nodes.
add_executable(image_pipeline_all_in_one
  src/image_pipeline/image_pipeline_all_in_one.cpp)
ament_target_dependencies(image_pipeline_all_in_one
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

# A single program with one of each of the image pipeline demo nodes, but two image views.
add_executable(image_pipeline_with_two_image_view
  src/image_pipeline/image_pipeline_with_two_image_view.cpp)
ament_target_dependencies(image_pipeline_with_two_image_view
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

# A stand alone node which produces images from a camera using OpenCV.
add_executable(camera_node
  src/image_pipeline/camera_node.cpp)
ament_target_dependencies(camera_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

# A stand alone node which adds some text to an image using OpenCV before passing it along.
add_executable(watermark_node
  src/image_pipeline/watermark_node.cpp)
ament_target_dependencies(watermark_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

# A stand alone node which consumes images and displays them using OpenCV.
add_executable(image_view_node
  src/image_pipeline/image_view_node.cpp)
ament_target_dependencies(image_view_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

install(TARGETS
  two_node_pipeline
  cyclic_pipeline
  image_pipeline_all_in_one
  image_pipeline_with_two_image_view
  camera_node
  watermark_node
  image_view_node
  DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(rmw_implementation_cmake REQUIRED)

  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1:exe2.
  set(demo_tests
    cyclic_pipeline
    two_node_pipeline)

  macro(tests)
    foreach(demo_test ${demo_tests})
      string(REPLACE ":" ";" demo_executables "${demo_test}")
      set(RCLCPP_DEMOS_EXPECTED_OUTPUT "")
      foreach(executable ${demo_executables})
        list(APPEND RCLCPP_DEMOS_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
      endforeach()

      set(RCLCPP_DEMOS_EXECUTABLE "")
      foreach(executable ${demo_executables})
        list(APPEND RCLCPP_DEMOS_EXECUTABLE "$<TARGET_FILE:${executable}>")
      endforeach()

      string(REPLACE ";" "_" exe_list_underscore "${demo_executables}")
      configure_file(
        test/test_executables_demo.py.in
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      ament_add_nose_test(test_demo_${exe_list_underscore}${target_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        TIMEOUT 30
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation})
      set_tests_properties(test_demo_${exe_list_underscore}${target_suffix}
        PROPERTIES DEPENDS "test_demo_${exe_list_underscore}${target_suffix} test_demo_${exe_list_underscore}${target_suffix}")
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

ament_package()
