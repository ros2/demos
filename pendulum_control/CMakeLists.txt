cmake_minimum_required(VERSION 2.8.3)

project(pendulum_control)

if(APPLE OR WIN32)
  message(STATUS "The pendulum_control demo is only supported on Linux. Package will not be built.")
  return()
endif()

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rttest)

include_directories(include)

if(NOT rttest_FOUND)
  message(STATUS "rttest not found. pendulum_control package will not be built.")
  return()
endif()

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(pendulum_demo${target_suffix}
    src/pendulum_demo.cpp)
  ament_target_dependencies(pendulum_demo${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  add_executable(pendulum_logger${target_suffix}
    src/pendulum_logger.cpp)
  ament_target_dependencies(pendulum_logger${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  add_executable(pendulum_teleop${target_suffix}
    src/pendulum_teleop.cpp)
  ament_target_dependencies(pendulum_teleop${target_suffix}
    "pendulum_msgs"
    "rclcpp${target_suffix}"
    "rttest")

  install(TARGETS
    pendulum_demo${target_suffix}
    pendulum_logger${target_suffix}
    pendulum_teleop${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

install(
  DIRECTORY scripts/
  DESTINATION bin
)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Linux-specific executable extensions logic because other OSes are disabled at the top of this
  # file.
  set(test_executable_extension "")
  set(test_executable_subfolder "")

  # get the rmw implementations ahead of time
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  set(RCLCPP_DEMO_PENDULUM_LOGGER_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/pendulum_logger")
  set(RCLCPP_DEMO_PENDULUM_DEMO_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/pendulum_demo")
  set(RCLCPP_DEMO_PENDULUM_DEMO_TELEOP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/pendulum_demo_teleop")
  set(RCLCPP_DEMO_PENDULUM_TELEOP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/pendulum_teleop")

  foreach(middleware_impl ${middleware_implementations})
    set(RCLCPP_DEMO_PENDULUM_LOGGER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/pendulum_logger__${middleware_impl}${test_executable_extension}")
    set(RCLCPP_DEMO_PENDULUM_DEMO_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/pendulum_demo__${middleware_impl}${test_executable_extension}")
    set(RCLCPP_DEMO_PENDULUM_TELEOP_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/pendulum_teleop__${middleware_impl}${test_executable_extension}")

    configure_file(
      test/test_pendulum_demo.py.in
      test_pendulum__${middleware_impl}.py
      @ONLY
    )

    configure_file(
      test/test_pendulum_teleop.py.in
      test_pendulum_teleop__${middleware_impl}.py
      @ONLY
    )

    ament_add_nose_test(test_demo_pendulum__${middleware_impl}
      "${CMAKE_CURRENT_BINARY_DIR}/test_pendulum__${middleware_impl}.py"
      TIMEOUT 20)
    set_tests_properties(test_demo_pendulum__${middleware_impl}
    PROPERTIES DEPENDS "test_demo_pendulum__${middleware_impl} test_demo_pendulum__${middleware_impl}")

    ament_add_nose_test(test_demo_pendulum_teleop__${middleware_impl}
      "${CMAKE_CURRENT_BINARY_DIR}/test_pendulum_teleop__${middleware_impl}.py"
      TIMEOUT 20)
    set_tests_properties(test_demo_pendulum_teleop__${middleware_impl}
    PROPERTIES DEPENDS "test_demo_pendulum_teleop__${middleware_impl} test_demo_pendulum_teleop__${middleware_impl}")

  endforeach()
endif()

ament_package()
