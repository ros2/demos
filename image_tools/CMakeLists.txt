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
find_package(rcutils REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(showimage_component SHARED
  src/showimage.cpp)
ament_target_dependencies(showimage_component
  "rclcpp"
  "sensor_msgs"
  "std_msgs"    
  "rclcpp_components"
  "OpenCV")
rclcpp_components_register_nodes(showimage_component "image_tools::ShowImage")

add_library(cam2image_component SHARED 
  src/cam2image.cpp)
ament_target_dependencies(cam2image_component
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "rclcpp_components")
rclcpp_components_register_nodes(cam2image_component "image_tools::Cam2Image")

add_executable(cam2image
  src/cam2image_main.cpp
  src/options.cpp
  src/burger.cpp)
target_link_libraries(cam2image
  showimage_component
  cam2image_component)
ament_target_dependencies(cam2image
  "rclcpp")

  add_executable(showimage
  src/showimage_main.cpp
  src/options.cpp)
target_link_libraries(showimage
  showimage_component
  cam2image_component)
ament_target_dependencies(showimage
  "rclcpp")

install(TARGETS
  showimage_component
  cam2image_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  showimage
  cam2image
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  # These are the regex's for validating the output of the executables.
  # set(RCLCPP_DEMO_SHOWIMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/showimage")
  set(RCLCPP_DEMO_CAM2IMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/cam2image")

  macro(testing_targets)
    set(RCLCPP_DEMO_CAM2IMAGE_EXECUTABLE $<TARGET_FILE:cam2image>)
    # set(RCLCPP_DEMO_SHOWIMAGE_EXECUTABLE $<TARGET_FILE:showimage>)

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
