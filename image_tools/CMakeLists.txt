cmake_minimum_required(VERSION 3.5)

project(image_tools)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(imagetools SHARED
  src/cam2image.cpp
  src/showimage.cpp
  src/burger.cpp)
target_compile_definitions(imagetools
  PRIVATE "IMAGE_TOOLS_BUILDING_DLL")
ament_target_dependencies(imagetools
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rclcpp_components"
  "OpenCV")
rclcpp_components_register_node(imagetools PLUGIN "image_tools::Cam2Image" EXECUTABLE cam2image)
rclcpp_components_register_node(imagetools PLUGIN "image_tools::ShowImage" EXECUTABLE showimage)

install(TARGETS
  imagetools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  # These are the regex's for validating the output of the executables.
  set(RCLCPP_DEMO_SHOWIMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/showimage")
  set(RCLCPP_DEMO_CAM2IMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/cam2image")

  macro(testing_targets)
    set(RCLCPP_DEMO_CAM2IMAGE_EXECUTABLE $<TARGET_FILE:cam2image>)
    set(RCLCPP_DEMO_SHOWIMAGE_EXECUTABLE $<TARGET_FILE:showimage>)

    configure_file(
      test/test_executables_demo.py.in
      test_showimage_cam2image${target_suffix}.py.genexp
      @ONLY
    )

    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_showimage_cam2image${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_showimage_cam2image${target_suffix}.py.genexp"
    )

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_showimage_cam2image${target_suffix}_$<CONFIG>.py"
      TARGET test_showimage_cam2image${target_suffix}
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation}
      TIMEOUT 30
    )
  endmacro()

  call_for_each_rmw_implementation(testing_targets)

endif()

ament_package()
