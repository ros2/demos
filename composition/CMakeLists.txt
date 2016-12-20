cmake_minimum_required(VERSION 3.5)

project(composition)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# get the rmw implementations ahead of time
find_package(rmw_implementation_cmake REQUIRED)
get_available_rmw_implementations(rmw_implementations)
foreach(rmw_implementation ${rmw_implementations})
  find_package("${rmw_implementation}" REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadNode.srv"
)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

macro(targets)
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

  add_library(talker_component${target_suffix} SHARED
    src/talker_component.cpp)
  target_compile_definitions(talker_component${target_suffix}
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(talker_component${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "class_loader")
  rclcpp_register_node_plugins(talker_component${target_suffix} "composition::Talker${target_suffix}")
  set(node_plugins "${node_plugins}composition::Talker${target_suffix};$<TARGET_FILE:talker_component${target_suffix}>\n")

  add_library(listener_component${target_suffix} SHARED
    src/listener_component.cpp)
  target_compile_definitions(listener_component${target_suffix}
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(listener_component${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "class_loader")
  rclcpp_register_node_plugins(listener_component${target_suffix} "composition::Listener${target_suffix}")
  set(node_plugins "${node_plugins}composition::Listener${target_suffix};$<TARGET_FILE:listener_component${target_suffix}>\n")

  add_library(server_component${target_suffix} SHARED
    src/server_component.cpp)
  target_compile_definitions(server_component${target_suffix}
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(server_component${target_suffix}
    "rclcpp${target_suffix}"
    "example_interfaces"
    "class_loader")
  rclcpp_register_node_plugins(server_component${target_suffix} "composition::Server${target_suffix}")
  set(node_plugins "${node_plugins}composition::Server${target_suffix};$<TARGET_FILE:server_component${target_suffix}>\n")

  add_library(client_component${target_suffix} SHARED
    src/client_component.cpp)
  target_compile_definitions(client_component${target_suffix}
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(client_component${target_suffix}
    "rclcpp${target_suffix}"
    "example_interfaces"
    "class_loader")
  rclcpp_register_node_plugins(client_component${target_suffix} "composition::Client${target_suffix}")
  set(node_plugins "${node_plugins}composition::Client${target_suffix};$<TARGET_FILE:client_component${target_suffix}>\n")

  add_executable(manual_composition${target_suffix}
    src/manual_composition.cpp)
  target_link_libraries(manual_composition${target_suffix}
    talker_component${target_suffix}
    listener_component${target_suffix}
    server_component${target_suffix}
    client_component${target_suffix})
  ament_target_dependencies(manual_composition${target_suffix}
    "rclcpp${target_suffix}")

  add_executable(linktime_composition${target_suffix}
    src/linktime_composition.cpp)
  set(libs
    talker_component${target_suffix}
    listener_component${target_suffix}
    server_component${target_suffix}
    client_component${target_suffix})
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(libs
      "-Wl,--no-as-needed"
      ${libs}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(linktime_composition${target_suffix} ${libs})
  ament_target_dependencies(linktime_composition${target_suffix}
    "rclcpp${target_suffix}"
    "class_loader")

  add_executable(dlopen_composition${target_suffix}
    src/dlopen_composition.cpp)
  ament_target_dependencies(dlopen_composition${target_suffix}
    "rclcpp${target_suffix}"
    "class_loader")

  add_executable(api_composition${target_suffix}
    src/api_composition.cpp)
  if(NOT "${target_suffix}" STREQUAL "")
    target_compile_definitions(api_composition${target_suffix}
      PRIVATE "RMW_IMPLEMENTATION_SUFFIX=${target_suffix}")
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(api_composition${target_suffix} "stdc++fs")
  endif()
  ament_target_dependencies(api_composition${target_suffix}
    "rclcpp${target_suffix}"
    "ament_index_cpp"
    "class_loader")
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(api_composition${target_suffix}
      ${PROJECT_NAME} ${typesupport_impl})
  endforeach()

  add_executable(api_composition_cli${target_suffix}
    src/api_composition_cli.cpp)
  ament_target_dependencies(api_composition_cli${target_suffix}
    "rclcpp${target_suffix}")
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(api_composition_cli${target_suffix}
      ${PROJECT_NAME} ${typesupport_impl})
  endforeach()

  install(TARGETS
    talker_component${target_suffix}
    listener_component${target_suffix}
    server_component${target_suffix}
    client_component${target_suffix}
    manual_composition${target_suffix}
    linktime_composition${target_suffix}
    dlopen_composition${target_suffix}
    api_composition${target_suffix}
    api_composition_cli${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GENERATE
    OUTPUT
    "test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
    CONTENT "${node_plugins}")

  macro(tests)
    set(MANUAL_COMPOSITION_EXECUTABLE $<TARGET_FILE:manual_composition${target_suffix}>)
    set(LINKTIME_COMPOSITION_EXECUTABLE $<TARGET_FILE:linktime_composition${target_suffix}>)
    set(DLOPEN_COMPOSITION_EXECUTABLE $<TARGET_FILE:dlopen_composition${target_suffix}>)
    set(TALKER_LIBRARY $<TARGET_FILE:talker_component${target_suffix}>)
    set(LISTENER_LIBRARY $<TARGET_FILE:listener_component${target_suffix}>)
    set(SERVER_LIBRARY $<TARGET_FILE:server_component${target_suffix}>)
    set(CLIENT_LIBRARY $<TARGET_FILE:client_component${target_suffix}>)
    set(API_COMPOSITION_EXECUTABLE $<TARGET_FILE:api_composition${target_suffix}>)
    set(API_COMPOSITION_CLI_EXECUTABLE $<TARGET_FILE:api_composition_cli${target_suffix}>)
    set(EXPECTED_OUTPUT_ALL "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_all")
    set(EXPECTED_OUTPUT_PUBSUB "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_pubsub")
    set(EXPECTED_OUTPUT_SRV "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_srv")

    configure_file(
      test/test_composition.py.in
      test_composition${target_suffix}.py.genexp
      @ONLY
    )
    file(GENERATE
      OUTPUT test_composition${target_suffix}_$<CONFIG>.py
      INPUT test_composition${target_suffix}.py.genexp)
    ament_add_nose_test(test_composition${target_suffix}
      "${CMAKE_CURRENT_BINARY_DIR}/test_composition${target_suffix}_$<CONFIG>.py"
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 90)
  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)
endif()

ament_package()
