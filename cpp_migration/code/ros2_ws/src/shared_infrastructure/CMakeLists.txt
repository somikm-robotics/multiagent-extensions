cmake_minimum_required(VERSION 3.8)
project(shared_infrastructure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(transport_interfaces REQUIRED)

add_library(shared_infrastructure SHARED
  source_files/agent_utils.cpp
)

ament_target_dependencies(shared_infrastructure
  geometry_msgs
  ament_index_cpp
)

target_include_directories(shared_infrastructure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(shared_infrastructure
  Boost::boost
  Boost::filesystem
)

#Plugins
# orbit controller plugin

add_library(nav2_orbit_controller_plugin SHARED
  source_files/nav2_orbit_controller_plugin.cpp
)

target_include_directories(nav2_orbit_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_orbit_controller_plugin
  nav2_core
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  rcl_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/orbit_controller_plugin/plugin.xml
)

install(TARGETS
  nav2_orbit_controller_plugin
  EXPORT export_nav2_orbit_controller_plugin
  LIBRARY DESTINATION lib
)


ament_export_libraries(nav2_orbit_controller_plugin)
ament_export_targets(export_nav2_orbit_controller_plugin HAS_LIBRARY_TARGET)

#dust plume sensor plugin
add_library(dust_plume_sensor_plugin SHARED
  source_files/dust_plume_sensor_plugin.cpp
)

target_include_directories(dust_plume_sensor_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dust_plume_sensor_plugin
                ignition-gazebo6 
                rclcpp 
                transport_interfaces)

target_link_libraries(dust_plume_sensor_plugin
  ignition-gazebo6)
 

install(TARGETS dust_plume_sensor_plugin
  LIBRARY DESTINATION lib
)


install(TARGETS shared_infrastructure
  EXPORT export_shared_infrastructure
  LIBRARY DESTINATION lib
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_include_directories(include)

ament_export_targets(export_shared_infrastructure HAS_LIBRARY_TARGET)

ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  rcl_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
  ament_index_cpp
  transport_interfaces
)



ament_package()
