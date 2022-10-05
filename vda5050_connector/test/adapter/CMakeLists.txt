# DEPENDENCIES
find_package(ament_cmake_gtest REQUIRED)

# MACRO
macro(add_dependencies_to_test target)
if (TARGET ${target})
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:common>)
  ament_target_dependencies(${target} ${cpp_dependencies} ${msg_dependencies})
  target_link_libraries(${target} ${library_adapter})
endif()
endmacro()

## LIBRARY TEST PLUGINS
set(plugins
  common/stub_nav_to_node.cpp
  common/stub_state_handlers.cpp
  common/stub_vda_actions.cpp
)
add_library(test_plugins SHARED ${plugins})
add_dependencies_to_test(test_plugins)

# TEST - NODE CONFIGURATION
ament_add_gtest(test_node_config node_config/node_config_test.cpp)
add_dependencies_to_test(test_node_config)

# TEST - PLUGIN LOAD
ament_add_gtest(test_plugin_load plugin_load/plugin_load_test.cpp)
add_dependencies_to_test(test_plugin_load)

# TEST - PLUGIN LOAD
ament_add_gtest(test_handler_execution handler_execution/handler_execution_test.cpp)
add_dependencies_to_test(test_handler_execution)

install(
  TARGETS test_plugins
  EXPORT test_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# EXPORT
ament_export_libraries(test_plugins)
ament_export_targets(test_plugins)
