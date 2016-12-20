cmake_minimum_required(VERSION 3.5)

project(image_tools)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(cam2image${target_suffix}
    src/burger.cpp
    src/cam2image.cpp
    src/options.cpp
  )
  ament_target_dependencies(cam2image${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "OpenCV")

  add_executable(showimage${target_suffix}
    src/options.cpp
    src/showimage.cpp
  )
  ament_target_dependencies(showimage${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "OpenCV")

  install(TARGETS
    cam2image${target_suffix}
    showimage${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # get the rmw implementations ahead of time
  get_available_rmw_implementations(rmw_implementations)
  foreach(rmw_implementation ${rmw_implementations})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  # These are the regex's for validating the output of the executables.
  set(RCLCPP_DEMO_SHOWIMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/showimage")
  set(RCLCPP_DEMO_CAM2IMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/cam2image")

  macro(testing_targets)
    if(
      # TODO(wjwwood): connext and connext_dynamic fail with:
      #
      #   COMMENDSrWriterService_write:!write. Reliable large data requires asynchronous writer.
      #
      # We can remove this disable for connext when that's fixed.
      "${rmw_implementation} " STREQUAL "rmw_connext_cpp " OR
      "${rmw_implementation} " STREQUAL "rmw_connext_dynamic_cpp " OR
      # TODO(wjwwood): fastrtps fails with:
      #
      #   uncaught exception of type std::runtime_error: vector overcomes the maximum length
      #
      # We can remove this disable for fastrtps when that's fixed.
      "${rmw_implementation} " STREQUAL "rmw_fastrtps_cpp "
    )
      message(STATUS "Skipping tests for '${rmw_implementation}'")
    else()
      set(RCLCPP_DEMO_CAM2IMAGE_EXECUTABLE $<TARGET_FILE:cam2image${target_suffix}>)
      set(RCLCPP_DEMO_SHOWIMAGE_EXECUTABLE $<TARGET_FILE:showimage${target_suffix}>)

      configure_file(
        test/test_executables_demo.py.in
        test_showimage_cam2image${target_suffix}.py.genexp
        @ONLY
      )
      file(GENERATE
        OUTPUT test_showimage_cam2image${target_suffix}_$<CONFIG>.py
        INPUT test_showimage_cam2image${target_suffix}.py.genexp)

      ament_add_nose_test(test_demo_showimage_cam2image${target_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_showimage_cam2image${target_suffix}_$<CONFIG>.py"
        TIMEOUT 60)
    endif()
  endmacro()

  call_for_each_rmw_implementation(testing_targets)

endif()

ament_package()
