cmake_minimum_required(VERSION 3.5)

project(image_tools)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
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
    src/cam2image.cpp
    src/options.cpp)
  ament_target_dependencies(cam2image${target_suffix}
    "rclcpp${target_suffix}"
    "sensor_msgs"
    "std_msgs"
    "OpenCV")

  add_executable(showimage${target_suffix}
    src/showimage.cpp
    src/options.cpp)
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

  if(NOT WIN32)
    set(test_executable_extension "")
    set(test_executable_subfolder "")
  else()
    set(test_executable_extension ".exe")
    set(test_executable_subfolder "/${CMAKE_BUILD_TYPE}")
    if("${test_executable_subfolder}" STREQUAL "/")
      set(test_executable_subfolder "/Debug")
    endif()
  endif()

  # get the rmw implementations ahead of time
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  set(RCLCPP_DEMO_SHOWIMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/showimage")
  set(RCLCPP_DEMO_CAM2IMAGE_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/cam2image")
  set(RCLCPP_DEMO_CAM2IMAGE_VIDEO "${CMAKE_CURRENT_SOURCE_DIR}/test/test_capture.mp4")

  foreach(middleware_impl ${middleware_implementations})
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impl "${middleware_impl}")

    set(RCLCPP_DEMO_SHOWIMAGE_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/showimage__${middleware_impl}${test_executable_extension}")
    set(RCLCPP_DEMO_CAM2IMAGE_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/cam2image__${middleware_impl}${test_executable_extension}")

    configure_file(
      test/test_executables_demo.py.in
      test_showimage_cam2image__${middleware_impl}.py
      @ONLY
    )

    ament_add_nose_test(test_demo_showimage_cam2image__${middleware_impl}
      "${CMAKE_CURRENT_BINARY_DIR}/test_showimage_cam2image__${middleware_impl}.py"
      TIMEOUT 30)
    set_tests_properties(test_demo_showimage_cam2image__${middleware_impl}
    PROPERTIES DEPENDS "test_demo_showimage_cam2image__${middleware_impl} test_demo_showimage_cam2image__${middleware_impl}")
  endforeach()
endif()

ament_package()
