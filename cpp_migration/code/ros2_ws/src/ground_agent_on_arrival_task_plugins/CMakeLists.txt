cmake_minimum_required(VERSION 3.8)
project(ground_agent_on_arrival_task_plugins)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(shared_infrastructure REQUIRED)
find_package(Boost REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ignition-math6 REQUIRED)

#Plugins
# initial inspection plugin
add_library(initial_inspection_plugin SHARED
  source_files/initial_inspection_plugin.cpp
)

target_include_directories(initial_inspection_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(initial_inspection_plugin
  rclcpp
  transport_interfaces
  pluginlib
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  shared_infrastructure
)

ament_add_gtest(initial_inspection_plugin_test
  test/initial_inspection_plugin_test.cpp
)

ament_target_dependencies(initial_inspection_plugin_test
  rclcpp
  transport_interfaces
  pluginlib
  sensor_msgs
  nav_msgs
  shared_infrastructure
  cv_bridge
  ament_index_cpp
)


target_link_libraries(initial_inspection_plugin_test
                      initial_inspection_plugin
                      ${OpenCV_LIBS}
                      )

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/initial_inspection_plugin/plugin.xml
)

install(TARGETS
  initial_inspection_plugin
  EXPORT export_initial_inspection_plugin
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(initial_inspection_plugin)
ament_export_targets(export_initial_inspection_plugin HAS_LIBRARY_TARGET)


# orbit hazard plugin
add_library(orbit_hazard_plugin SHARED
  source_files/orbit_hazard_manager_plugin.cpp
  source_files/twist_commander_orbit_hazard_plugin.cpp
  source_files/nav2_orbit_hazard_manager_plugin.cpp
)

target_include_directories(orbit_hazard_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orbit_hazard_plugin
  rclcpp
  transport_interfaces
  pluginlib
  nav_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs
  tf2_ros
  rcl_interfaces
  std_msgs
  shared_infrastructure
)

ament_add_gtest(orbit_hazard_manager_plugin_tests
  test/orbit_hazard_manager_plugin_tests.cpp
)

ament_target_dependencies(orbit_hazard_manager_plugin_tests
  rclcpp
  transport_interfaces
  pluginlib
  nav_msgs
  shared_infrastructure
  nav2_msgs
  std_msgs
  rclcpp_action
  cv_bridge
  ament_index_cpp
)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/orbit_hazard_plugin/plugin.xml
)

install(TARGETS
  orbit_hazard_plugin
  EXPORT export_orbit_hazard_plugin
  LIBRARY DESTINATION lib
)

ament_export_libraries(orbit_hazard_plugin)
ament_export_targets(export_orbit_hazard_plugin HAS_LIBRARY_TARGET)

ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  transport_interfaces
  pluginlib
  sensor_msgs                                                                                                                                                                
  geometry_msgs
  nav_msgs
  cv_bridge
  shared_infrastructure
  tf2_ros
  rcl_interfaces
  nav2_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

message(STATUS "✅ Ground Agent Plugins for ground_agent_on_arrival_task_plugins CMakeLists.txt is running")


ament_package()


