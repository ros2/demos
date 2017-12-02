cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp_native)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_fastrtps_cpp QUIET)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "std_msgs"
    "rmw_fastrtps_cpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

if(rmw_fastrtps_cpp_FOUND)
  find_package(rmw_fastrtps_cpp REQUIRED)
  custom_executable(talker)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_pytest REQUIRED)

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

    ament_add_pytest_test(test_tutorial_${exe_list_underscore}
      "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}_$<CONFIG>.py"
      TIMEOUT 30
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=rmw_fastrtps_cpp
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
    foreach(executable ${tutorial_executables})
      set_property(
        TEST test_tutorial_${exe_list_underscore}
        APPEND PROPERTY DEPENDS ${executable})
    endforeach()
  endif()
endif()

ament_package()
