cmake_minimum_required(VERSION 3.5)

project(barcode_reader)

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

find_package(barcode_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# zbar needs pkgconfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED IMPORTED_TARGET zbar)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(scanner2 SHARED
  src/scanner2.cpp)
target_compile_definitions(scanner2
  PRIVATE "COMPOSITION_BUILDING_DLL")
# Non-ament packages cannot be used in ament_target_dependencies.
target_link_libraries(scanner2
  PkgConfig::ZBar
)
ament_target_dependencies(scanner2
  example_interfaces
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  barcode_msgs
  OpenCV)
rclcpp_components_register_nodes(scanner2 "composition::Scanner2")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(scanner
  src/scanner.cpp)
target_link_libraries(scanner
  scanner2)
ament_target_dependencies(scanner
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  barcode_msgs
  OpenCV)

set(libs
  scanner2)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

install(TARGETS
  scanner2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  scanner
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
    set(SCANNER_EXECUTABLE $<TARGET_FILE:scanner>)
    set(CLIENT_LIBRARY $<TARGET_FILE:scanner2>)
    set(EXPECTED_OUTPUT_ALL "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_all")
    set(EXPECTED_OUTPUT_PUBSUB "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_pubsub")
    set(EXPECTED_OUTPUT_SRV "${CMAKE_CURRENT_SOURCE_DIR}/test/composition_srv")

    set(test_names
      "test_scanner"
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
