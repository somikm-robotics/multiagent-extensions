cmake_minimum_required(VERSION 3.8)
project(ground_agent_mission_handler_plugins)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_core REQUIRED)
find_package(shared_infrastructure REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ament_cmake_gtest REQUIRED)
find_package(ignition-math6 REQUIRED)

include_directories(
  include
)

#Plugins
# navigation plugin
add_library(navigation_plugin SHARED
  source_files/nav2_navigation_plugin.cpp
)

target_include_directories(navigation_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navigation_plugin
  rclcpp
  transport_interfaces
  pluginlib
  nav2_msgs
  rclcpp_action
  shared_infrastructure
)

ament_add_gtest(nav2_navigation_plugin_test
  test/nav2_navigation_plugin_test.cpp
)

ament_target_dependencies(nav2_navigation_plugin_test
  rclcpp
  transport_interfaces
  pluginlib
  nav2_msgs
  rclcpp_action
  shared_infrastructure
)

target_link_libraries(nav2_navigation_plugin_test
                      navigation_plugin)


pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/navigation_plugin/plugin.xml
)

install(TARGETS
  navigation_plugin
  EXPORT export_navigation_plugin
  LIBRARY DESTINATION lib
)


ament_export_libraries(navigation_plugin)
ament_export_targets(export_navigation_plugin HAS_LIBRARY_TARGET)

# path planning plugin
add_library(path_planner_plugin SHARED
  source_files/path_planner_plugin.cpp
  source_files/nav2_path_planner_plugin.cpp
  source_files/a_star_path_planner_plugin.cpp
)

target_include_directories(path_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_planner_plugin
  rclcpp
  transport_interfaces
  pluginlib
  nav2_msgs
  rclcpp_action
  nav2_core
  shared_infrastructure
)

ament_add_gtest(nav2_path_planner_plugin_test
  test/nav2_path_planner_plugin_test.cpp
)

ament_target_dependencies(nav2_path_planner_plugin_test
  rclcpp
  transport_interfaces
  pluginlib
  nav2_msgs
  rclcpp_action
  shared_infrastructure
)

target_link_libraries(nav2_path_planner_plugin_test
                      path_planner_plugin)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/path_planner_plugin/plugin.xml
)

install(TARGETS
  path_planner_plugin
  EXPORT export_path_planner_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)


ament_export_libraries(path_planner_plugin)
ament_export_targets(export_path_planner_plugin HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  transport_interfaces
  pluginlib
  nav2_msgs
  nav2_core
  rclcpp_action
  shared_infrastructure
)

message(STATUS "✅ Ground Agent - ground_agent_mission_handler_plugins CMakeLists.txt is running")


ament_package()


