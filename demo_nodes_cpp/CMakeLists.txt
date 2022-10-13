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

function(add_demo_dependencies library)
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
custom_executable(parameters parameter_event_handler)
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
  src/topics/content_filtering_publisher.cpp
  src/topics/content_filtering_subscriber.cpp
  src/topics/talker.cpp
  src/topics/talker_loaned_message.cpp
  src/topics/talker_serialized_message.cpp
  src/topics/listener.cpp
  src/topics/listener_serialized_message.cpp
  src/topics/listener_best_effort.cpp)
add_demo_dependencies(timers_library)
add_demo_dependencies(services_library)
add_demo_dependencies(parameters_library)
add_demo_dependencies(topics_library)

rclcpp_components_register_node(timers_library
  PLUGIN "demo_nodes_cpp::OneOffTimerNode"
  EXECUTABLE one_off_timer)
rclcpp_components_register_node(timers_library
  PLUGIN "demo_nodes_cpp::ReuseTimerNode"
  EXECUTABLE reuse_timer)

rclcpp_components_register_node(services_library
  PLUGIN "demo_nodes_cpp::ServerNode"
  EXECUTABLE add_two_ints_server)
rclcpp_components_register_node(services_library
  PLUGIN "demo_nodes_cpp::ClientNode"
  EXECUTABLE add_two_ints_client_async)

rclcpp_components_register_node(parameters_library
  PLUGIN "demo_nodes_cpp::ListParameters"
  EXECUTABLE list_parameters)
rclcpp_components_register_node(parameters_library
  PLUGIN "demo_nodes_cpp::ParameterBlackboard"
  EXECUTABLE parameter_blackboard)
rclcpp_components_register_node(parameters_library
  PLUGIN "demo_nodes_cpp::SetAndGetParameters"
  EXECUTABLE set_and_get_parameters)
rclcpp_components_register_node(parameters_library
  PLUGIN "demo_nodes_cpp::ParameterEventsAsyncNode"
  EXECUTABLE parameter_events_async)
rclcpp_components_register_node(parameters_library
  PLUGIN "demo_nodes_cpp::EvenParameterNode"
  EXECUTABLE even_parameters_node)

rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::ContentFilteringPublisher"
  EXECUTABLE content_filtering_publisher)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::ContentFilteringSubscriber"
  EXECUTABLE content_filtering_subscriber)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::LoanedMessageTalker"
  EXECUTABLE talker_loaned_message)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::SerializedMessageTalker"
  EXECUTABLE talker_serialized_message)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::Listener"
  EXECUTABLE listener)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::SerializedMessageListener"
  EXECUTABLE listener_serialized_message)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::ListenerBestEffort"
  EXECUTABLE listener_best_effort)

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
    "content_filtering_publisher:content_filtering_subscriber"
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
