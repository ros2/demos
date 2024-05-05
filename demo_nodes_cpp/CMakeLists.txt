cmake_minimum_required(VERSION 3.12)

project(demo_nodes_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable subfolder target)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES" ${ARGN})
  add_executable(${target} src/${subfolder}/${target}.cpp)
  target_include_directories(${target} PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${target} PRIVATE
    ${ARG_DEPENDENCIES}
  )
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# TODO(clalancette): libc++ 14 (in Ubuntu 22.04) does not support
# Polymorphic memory resources: https://en.cppreference.com/w/cpp/compiler_support/17
# So we use the old custom allocator clang for now; we can eventually remove this.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 16.0.0)
  set(allocator_file allocator_tutorial_custom)
else()
  set(allocator_file allocator_tutorial_pmr)
endif()
add_executable(allocator_tutorial src/topics/${allocator_file}.cpp)
target_include_directories(allocator_tutorial PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(allocator_tutorial PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
install(TARGETS allocator_tutorial
  DESTINATION lib/${PROJECT_NAME})

custom_executable(services add_two_ints_client
  DEPENDENCIES ${example_interfaces_TARGETS} rclcpp::rclcpp)

custom_executable(parameters list_parameters_async
  DEPENDENCIES rclcpp::rclcpp)

custom_executable(parameters parameter_events
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp)

custom_executable(parameters parameter_event_handler
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp)

custom_executable(parameters set_and_get_parameters_async
  DEPENDENCIES rclcpp::rclcpp)

custom_executable(events matched_event_detect
  DEPENDENCIES rclcpp::rclcpp ${std_msgs_TARGETS})

custom_executable(logging use_logger_service
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp ${std_msgs_TARGETS})

function(create_demo_library plugin executable)
  cmake_parse_arguments(ARG "" "" "FILES;DEPENDENCIES" ${ARGN})
  set(library ${executable}_library)
  add_library(${library} SHARED ${ARG_FILES})
  target_compile_definitions(${library}
    PRIVATE "DEMO_NODES_CPP_BUILDING_DLL")
  target_include_directories(${library} PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${library} PRIVATE
    ${ARG_DEPENDENCIES}
  )
  install(TARGETS ${library}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  set(DEMO_EXECUTOR "rclcpp::executors::SingleThreadedExecutor" CACHE STRING "The executor for the demo nodes")
  message(STATUS "Setting executor of ${executable} to ${DEMO_EXECUTOR}")
  rclcpp_components_register_node(${library}
    PLUGIN ${plugin}
    EXECUTABLE ${executable}
    EXECUTOR ${DEMO_EXECUTOR})
endfunction()

# Timers
create_demo_library("demo_nodes_cpp::OneOffTimerNode" one_off_timer
  FILES src/timers/one_off_timer.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::ReuseTimerNode" reuse_timer
  FILES src/timers/reuse_timer.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component)

# Parameters
create_demo_library("demo_nodes_cpp::ListParameters" list_parameters
  FILES src/parameters/list_parameters.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::ParameterBlackboard" parameter_blackboard
  FILES src/parameters/parameter_blackboard.cpp
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::SetAndGetParameters" set_and_get_parameters
  FILES src/parameters/set_and_get_parameters.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::ParameterEventsAsyncNode" parameter_events_async
  FILES src/parameters/parameter_events_async.cpp
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::EvenParameterNode" even_parameters_node
  FILES src/parameters/even_parameters_node.cpp
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::SetParametersCallback" set_parameters_callback
  FILES src/parameters/set_parameters_callback.cpp
  DEPENDENCIES ${rcl_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)

# Services
create_demo_library("demo_nodes_cpp::ServerNode" add_two_ints_server
  FILES src/services/add_two_ints_server.cpp
  DEPENDENCIES ${example_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::ClientNode" add_two_ints_client_async
  FILES src/services/add_two_ints_client_async.cpp
  DEPENDENCIES ${example_interfaces_TARGETS} rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::IntrospectionServiceNode" introspection_service
  FILES src/services/introspection_service.cpp
  DEPENDENCIES ${example_interfaces_TARGETS} ${rcl_interfaces_TARGETS} rcl::rcl rclcpp::rclcpp rclcpp_components::component)
create_demo_library("demo_nodes_cpp::IntrospectionClientNode" introspection_client
  FILES src/services/introspection_client.cpp
  DEPENDENCIES ${example_interfaces_TARGETS} ${rcl_interfaces_TARGETS} rcl::rcl rclcpp::rclcpp rclcpp_components::component)

# Topics
create_demo_library("demo_nodes_cpp::ContentFilteringPublisher" content_filtering_publisher
  FILES src/topics/content_filtering_publisher.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::ContentFilteringSubscriber" content_filtering_subscriber
  FILES src/topics/content_filtering_subscriber.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component rcpputils::rcpputils ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::Talker" talker
  FILES src/topics/talker.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::LoanedMessageTalker" talker_loaned_message
  FILES src/topics/talker_loaned_message.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::SerializedMessageTalker" talker_serialized_message
  FILES src/topics/talker_serialized_message.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::Listener" listener
  FILES src/topics/listener.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::SerializedMessageListener" listener_serialized_message
  FILES src/topics/listener_serialized_message.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})
create_demo_library("demo_nodes_cpp::ListenerBestEffort" listener_best_effort
  FILES src/topics/listener_best_effort.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
  set(tutorial_tests
    "content_filtering_publisher@publish_ms=100:content_filtering_subscriber"
    list_parameters_async
    list_parameters
    parameter_events_async
    parameter_events
    set_and_get_parameters_async
    set_and_get_parameters
    matched_event_detect
    use_logger_service
    "talker:listener"
  )
  set(service_tutorial_tests
    "add_two_ints_server@one_shot=True:add_two_ints_client"
    "add_two_ints_server@one_shot=True:add_two_ints_client_async"
  )

  macro(tests)
    set(tutorial_tests_to_test ${tutorial_tests})
    list(APPEND tutorial_tests_to_test ${service_tutorial_tests})

    foreach(tutorial_test ${tutorial_tests_to_test})
      set(DEMO_NODES_CPP_EXPECTED_OUTPUT "")
      set(DEMO_NODES_CPP_EXECUTABLE "")
      set(exe_list "")

      # We're expecting each tutorial_test to be of the form:
      #
      # exe1[@param1=value1@param2=value2][:exe2[@param3=value3@param4=value4]]
      #
      # That is, you can have one or more executables, and each of those
      # executables can be passed zero or more parameters.
      #
      # Unfortunately, we need to parse this list here in CMake since we need
      # to know the executable name so we can generate an absolute path.

      # Convert the colon-separated list we were given to a semi-colon
      # separated one so we can iterate over it in CMake.
      string(REPLACE ":" ";" exes_and_params "${tutorial_test}")
      foreach(exe_and_params ${exes_and_params})
        string(REPLACE "@" ";" params ${exe_and_params})
        # It is expected that the first variable is the executable
        list(GET params 0 executable)
        list(APPEND DEMO_NODES_CPP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
        list(APPEND exe_list ${executable})
        set(target_file "$<TARGET_FILE:${executable}>")

        # If there is anything left in the list at this time, assume they
        # are parameters.
        list(LENGTH params list_length)
        if(${list_length} GREATER 1)
          list(SUBLIST params 1 -1 params_only)
          string(JOIN "@" target_file ${target_file} ${params_only})
        endif()

        # This is what will get substituted into test_executables_tutorial.py.in below.
        # It looks exactly the same as exes_and_params, with the exception
        # that the executable names have been substituted for the absolute paths.
        list(APPEND DEMO_NODES_CPP_EXECUTABLE ${target_file})
      endforeach()
      string(REPLACE ";" "_" exe_list_underscore "${exe_list}")
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
      foreach(executable ${exe_list})
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
