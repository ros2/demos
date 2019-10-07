cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "DEMO_NODES_CPP_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils"
    "std_msgs")
endfunction()
# Tutorials of Publish/Subscribe with Topics
custom_executable(topics allocator_tutorial)

# Tutorials of Request/Response with Services
custom_executable(services add_two_ints_client)

# Tutorials of Parameters with Asynchronous and Synchronous
custom_executable(parameters list_parameters_async)
custom_executable(parameters parameter_events)
custom_executable(parameters set_and_get_parameters_async)

add_library(timers_library SHARED
  src/timers/one_off_timer.cpp
  src/timers/reuse_timer.cpp)
add_library(services_library SHARED
  src/services/add_two_ints_server.cpp
  src/services/add_two_ints_client.cpp
  src/services/add_two_ints_client_async.cpp)
add_library(parameters_library SHARED
  src/parameters/list_parameters.cpp
  src/parameters/parameter_blackboard.cpp
  src/parameters/set_and_get_parameters.cpp
  src/parameters/parameter_events_async.cpp
  src/parameters/even_parameters_node.cpp)
add_library(topics_library SHARED
  src/topics/talker.cpp
  src/topics/talker_serialized_message.cpp
  src/topics/listener.cpp
  src/topics/listener_serialized_message.cpp
  src/topics/listener_best_effort.cpp)
add_dependencies(timers_library)
add_dependencies(services_library)
add_dependencies(parameters_library)
add_dependencies(topics_library)

rclcpp_components_register_nodes(
  PLUGINS
    "demo_nodes_cpp::ReuseTimerNode"
    "demo_nodes_cpp::OneOffTimerNode"
  FROM timers_library
)

rclcpp_components_add_executable(one_off_timer
  PLUGINS "demo_nodes_cpp::ReuseTimerNode"
  FROM timers_library
)

rclcpp_components_add_executable(reuse_timer
  PLUGINS "demo_nodes_cpp::ReuseTimerNode"
  FROM timers_library
)

install(TARGETS
  one_off_timer reuse_timer
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(
  PLUGINS
    "demo_nodes_cpp::ServerNode"
    "demo_nodes_cpp::ClientNode"
  FROM services_library
)
rclcpp_components_add_executable(add_two_ints_server
  PLUGINS "demo_nodes_cpp::ServerNode"
  FROM services_library
)
rclcpp_components_add_executable(add_two_ints_client_async
  PLUGINS "demo_nodes_cpp::ClientNode"
  FROM services_library
)

install(TARGETS
  add_two_ints_server add_two_ints_client_async
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(
  PLUGINS
    "demo_nodes_cpp::ListParameters"
    "demo_nodes_cpp::ParameterBlackboard"
    "demo_nodes_cpp::SetAndGetParameters"
    "demo_nodes_cpp::ParameterEventsAsyncNode"
    "demo_nodes_cpp::EvenParameterNode"
  FROM parameters_library
)

rclcpp_components_add_executable(list_parameters
  PLUGINS "demo_nodes_cpp::ListParameters"
  FROM parameters_library
)

rclcpp_components_add_executable(parameter_blackboard
  PLUGINS "demo_nodes_cpp::ParameterBlackboard"
  FROM parameters_library
)

# rclcpp_components_add_executable(compound_server
#   PLUGINS
#     "demos_nodes_cpp::ServerNode"
#   FROM services_library
# )
#
# rclcpp_components_target_compose(compound_server
#   PLUGINS
#     "demos_nodes_cpp::ParameterBlackboard"
#   FROM parameters_library
# )

rclcpp_components_add_executable(set_and_get_parameters
  PLUGINS "demo_nodes_cpp::SetAndGetParameters"
  FROM parameters_library
)

rclcpp_components_add_executable(parameter_events_async
  PLUGINS "demo_nodes_cpp::ParameterEventsAsyncNode"
  FROM parameters_library
)

rclcpp_components_add_executable(even_parameters_node
  PLUGINS "demo_nodes_cpp::EvenParameterNode"
  FROM parameters_library
)

install(TARGETS
  list_parameters parameter_blackboard set_and_get_parameters
  parameter_events_async even_parameters_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(
  PLUGINS
    "demo_nodes_cpp::Talker"
    "demo_nodes_cpp::SerializedMessageTalker"
    "demo_nodes_cpp::Listener"
    "demo_nodes_cpp::SerializedMessageListener"
    "demo_nodes_cpp::ListenerBestEffort"
  FROM topics_library
)

rclcpp_components_add_executable(talker
  PLUGINS "demo_nodes_cpp::Talker"
  FROM topics_library
)

# rclcpp_components_add_executable(talker_and_listener
#   PLUGINS
#     "demos_nodes_cpp::Talker"
#     "demos_nodes_cpp::Listener"
#   FROM topics_library
# )

rclcpp_components_add_executable(talker_serialized_message
  PLUGINS "demo_nodes_cpp::SerializedMessageTalker"
  FROM topics_library
)

rclcpp_components_add_executable(listener
  PLUGINS "demo_nodes_cpp::Listener"
  FROM topics_library
)

rclcpp_components_add_executable(listener_serialized_message
  PLUGINS "demo_nodes_cpp::SerializedMessageListener"
  FROM topics_library
)

rclcpp_components_add_executable(listener_best_effort
  PLUGINS "demo_nodes_cpp::ListenerBestEffort"
  FROM topics_library
)

install(TARGETS
  talker talker_serialized_message
  listener listener_serialized_message
  listener_best_effort
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  timers_library
  services_library
  parameters_library
  topics_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
  set(tutorial_tests
    list_parameters_async
    list_parameters
    parameter_events_async
    parameter_events
    set_and_get_parameters_async
    set_and_get_parameters
    "talker:listener")
  set(service_tutorial_tests
    "add_two_ints_server:add_two_ints_client"
    "add_two_ints_server:add_two_ints_client_async"
  )

  macro(tests)
    set(tutorial_tests_to_test ${tutorial_tests})
    list(APPEND tutorial_tests_to_test ${service_tutorial_tests})

    foreach(tutorial_test ${tutorial_tests_to_test})
      string(REPLACE ":" ";" tutorial_executables "${tutorial_test}")
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
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      add_launch_test(
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        TARGET test_tutorial_${exe_list_underscore}${target_suffix}
        TIMEOUT 60
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation}
      )
      foreach(executable ${tutorial_executables})
        set_property(
          TEST test_tutorial_${exe_list_underscore}${target_suffix}
          APPEND PROPERTY DEPENDS ${executable}${target_suffix})
      endforeach()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
