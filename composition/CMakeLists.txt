cmake_minimum_required(VERSION 3.5)

project(composition)

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
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(listener_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(listener_component "composition::Listener")
set(node_plugins "${node_plugins}composition::Listener;$<TARGET_FILE:listener_component>\n")

add_library(node_like_listener_component SHARED
  src/node_like_listener_component.cpp)
target_compile_definitions(node_like_listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(node_like_listener_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(node_like_listener_component "composition::NodeLikeListener")
set(node_plugins "${node_plugins}composition::NodeLikeListener;$<TARGET_FILE:node_like_listener_component>\n")

add_library(server_component SHARED
  src/server_component.cpp)
target_compile_definitions(server_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "example_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(server_component "composition::Server")
set(node_plugins "${node_plugins}composition::Server;$<TARGET_FILE:server_component>\n")

add_library(client_component SHARED
  src/client_component.cpp)
target_compile_definitions(client_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(client_component
  "example_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(client_component "composition::Client")
set(node_plugins "${node_plugins}composition::Client;$<TARGET_FILE:client_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition
  talker_component
  listener_component
  server_component
  client_component)
ament_target_dependencies(manual_composition
  "rclcpp")

add_executable(linktime_composition
  src/linktime_composition.cpp)
set(libs
  talker_component
  listener_component
  server_component
  client_component)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()
target_link_libraries(linktime_composition ${libs})
ament_target_dependencies(linktime_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

add_executable(dlopen_composition
  src/dlopen_composition.cpp)
ament_target_dependencies(dlopen_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

install(TARGETS
  talker_component
  listener_component
  node_like_listener_component
  server_component
  client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  manual_composition
  linktime_composition
  dlopen_composition
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  file(GENERATE
    OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
    CONTENT "${node_plugins}")

  set(generated_python_files)
  macro(tests)
    set(MANUAL_COMPOSITION_EXECUTABLE $<TARGET_FILE:manual_composition>)
    set(LINKTIME_COMPOSITION_EXECUTABLE $<TARGET_FILE:linktime_composition>)
    set(DLOPEN_COMPOSITION_EXECUTABLE $<TARGET_FILE:dlopen_composition>)
    set(TALKER_LIBRARY $<TARGET_FILE:talker_component>)
    set(LISTENER_LIBRARY $<TARGET_FILE:listener_component>)
    set(SERVER_LIBRARY $<TARGET_FILE:server_component>)
    set(CLIENT_LIBRARY $<TARGET_FILE:client_component>)
    set(NODE_LIKE_LISTENER_LIBRARY $<TARGET_FILE:node_like_listener_component>)
    set(EXPECTED_OUTPUT_ALL "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_all")
    set(EXPECTED_OUTPUT_PUBSUB "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_pubsub")
    set(EXPECTED_OUTPUT_SRV "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_srv")

    set(test_names
      "test_dlopen_composition"
      "test_linktime_composition"
      "test_manual_composition"
    )
    foreach(test_name IN LISTS test_names)
      configure_file(
        test/${test_name}.py.in
        ${test_name}${target_suffix}.py.genexp
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
      )
      add_launch_test(
        "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
        TARGET ${test_name}${target_suffix}
        ENV RMW_IMPLEMENTATION=${rmw_implementation}
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        TIMEOUT 60
      )
      list(
        APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py")
    endforeach()
  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    # the generated code might contain longer lines for templated types
    MAX_LINE_LENGTH 999
    ${generated_python_files})
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
