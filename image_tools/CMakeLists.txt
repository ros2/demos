cmake_minimum_required(VERSION 2.8.3)

project(image_tools)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include)

get_available_rmw_implementations(middleware_implementations)
foreach(middleware_impl ${middleware_implementations})
  # Find package each of them.
  find_package("${middleware_impl}" REQUIRED)
endforeach()

function(add_executable_for_each_rmw_impl executable)
  # Build the executable for the default rmw implementation.
  add_executable(${executable} ${ARGN})
  ament_target_dependencies(${executable}
    "rclcpp"
    "rmw_implementation"
    "sensor_msgs"
    "std_msgs"
    "OpenCV"
  )

  install(TARGETS ${executable} DESTINATION bin)

  # Build an executable for each rmw implementation.
  foreach(middleware_impl_tmp ${middleware_implementations})
    add_executable(${executable}__${middleware_impl_tmp} ${ARGN})
    ament_target_dependencies(${executable}__${middleware_impl_tmp}
      "rclcpp"
      "${middleware_impl_tmp}"
      "sensor_msgs"
      "std_msgs"
      "OpenCV"
    )

    install(TARGETS ${executable}__${middleware_impl_tmp} DESTINATION bin)
  endforeach()

endfunction()

add_executable_for_each_rmw_impl(cam2image src/cam2image.cpp src/options.cpp)
add_executable_for_each_rmw_impl(showimage src/showimage.cpp src/options.cpp)

ament_package()
