cmake_minimum_required(VERSION 3.8)
project(vda5050_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SETS

set(cpp_dependencies
  rclcpp
  rclcpp_action
  pluginlib
)

set(msg_dependencies
  vda5050_msgs
  geometry_msgs
)

set(library_adapter adapter)

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(vda5050_msgs REQUIRED)

# BUILD

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateThroughNodes.action"
  "action/NavigateToNode.action"
  "action/ProcessVDAAction.action"
  "srv/GetState.srv"
  "srv/SupportedActions.srv"
  DEPENDENCIES vda5050_msgs
  tf2
  ADD_LINTER_TESTS
)

## EXECUTABLES
add_library(${library_adapter} src/adapter.cpp src/utils.cpp)
target_compile_features(${library_adapter} PUBLIC cxx_std_17)  # Require C++17
target_include_directories(${library_adapter} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${library_adapter}
  ${cpp_dependencies}
  ${msg_dependencies}
  tf2
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${library_adapter} "${cpp_typesupport_target}")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${library_adapter} PRIVATE "VDA5050_CONNECTOR_BUILDING_LIBRARY")

# INSTALL

## HEADERS
install(
  DIRECTORY include/
  DESTINATION include
)

## LIBRARY
install(
  TARGETS ${library_adapter}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## PYTHON MODULES
ament_python_install_package(${PROJECT_NAME}_py)

## PYTHON EXECUTABLES
install(PROGRAMS
  scripts/mqtt_bridge.py
  scripts/vda5050_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

## LAUNCH AND CONFIGURATION FILES
install(DIRECTORY
config
launch
DESTINATION share/${PROJECT_NAME}/)

# TEST

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_clang_format("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
  ament_flake8("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.flake8)
  ament_pep257()
  ament_copyright()

  set(_pytest_tests
    test/test_vda5050_controller.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
  add_subdirectory(test/adapter)
  pluginlib_export_plugin_description_file(vda5050_connector test/adapter/plugins_test.xml)
endif()

# EXPORT

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${library_adapter}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
