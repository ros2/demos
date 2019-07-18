cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

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

function(use_composition library folder executable)
  add_library(${library} SHARED
    src/${folder}/${executable}.cpp)
  ament_target_dependencies(${library}
    "rclcpp"
    "std_msgs"
    "rclcpp_components"
    "rcutil"
    "example_interfaces")
endfunction()
# Tutorials of Publish/Subscribe with Topics
custom_executable(topics allocator_tutorial)

# Tutorials of Request/Response with Services
custom_executable(services add_two_ints_client)

# Tutorials of Parameters with Asynchronous and Synchronous
custom_executable(parameters list_parameters_async)
custom_executable(parameters parameter_events)
custom_executable(parameters even_parameters_node)
custom_executable(parameters set_and_get_parameters_async)
custom_executable(parameters parameter_events_async)


use_composition(serverlib services add_two_ints_server)
rclcpp_components_register_node(serverlib PLUGIN "demo_nodes_cpp::ServerNode"
  EXECUTABLE add_two_ints_server)
use_composition(clientasynclib services add_two_ints_client_async)
rclcpp_components_register_node(clientasynclib PLUGIN "demo_nodes_cpp::ClientNode"
  EXECUTABLE add_two_ints_client_async)
use_composition(listparameterslib parameters list_parameters)
rclcpp_components_register_node(listparameterslib PLUGIN "demo_nodes_cpp::ListParameters"
  EXECUTABLE list_parameters)
use_composition(parameterblackboardlib parameters parameter_blackboard)
rclcpp_components_register_node(parameterblackboardlib PLUGIN "demo_nodes_cpp::ParameterBlackboard"
  EXECUTABLE parameter_blackboard)
use_composition(setandgetparameterslib parameters set_and_get_parameters)
rclcpp_components_register_node(setandgetparameterslib PLUGIN "demo_nodes_cpp::SetAndGetParameters" 
  EXECUTABLE set_and_get_parameters) 
use_composition(talkerlib topics talker)
rclcpp_components_register_node(talkerlib PLUGIN "demo_nodes_cpp::Talker"
  EXECUTABLE talker)
use_composition(talkerserializedlib topics talker_serialized_message)
rclcpp_components_register_node(talkerserializedlib PLUGIN "demo_nodes_cpp::SerializedMessageTalker"
  EXECUTABLE talker_serialized_message)
use_composition(listenerlib topics listener)
rclcpp_components_register_node(listenerlib PLUGIN "demo_nodes_cpp::Listener"
  EXECUTABLE listener)
use_composition(listenerserializedlib topics listener_serialized_message)
rclcpp_components_register_node(listenerserializedlib PLUGIN "demo_nodes_cpp::SerializedMessageListener"
  EXECUTABLE listener_serialized_message)
use_composition(listenerbesteffortlib topics listener_best_effort)
rclcpp_components_register_node(listenerbesteffortlib PLUGIN "demo_nodes_cpp::ListenerBestEffort"
  EXECUTABLE listener_best_effort)
use_composition(oneofftimerlib timers one_off_timer)
rclcpp_components_register_node(oneofftimerlib PLUGIN "demo_nodes_cpp::OneOffTimerNode"
  EXECUTABLE one_off_timer)
use_composition(resusetimerlib timers reuse_timer)
rclcpp_components_register_node(resusetimerlib PLUGIN "demo_nodes_cpp::ReuseTimerNode"
  EXECUTABLE reuse_timer)

install(TARGETS
  talkerlib
  serverlib
  clientasynclib
  talkerserializedlib
  listenerbesteffortlib
  listenerlib
  oneofftimerlib
  resusetimerlib
  parameterblackboardlib
  listparameterslib
  setandgetparameterslib
  listenerserializedlib
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
