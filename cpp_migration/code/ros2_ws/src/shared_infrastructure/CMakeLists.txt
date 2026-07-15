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
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

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
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ignition-math6 REQUIRED)

# shared infrastructure
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

install(TARGETS shared_infrastructure
  EXPORT export_shared_infrastructure
  LIBRARY DESTINATION lib
)

ament_export_targets(export_shared_infrastructure HAS_LIBRARY_TARGET)


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

#test helpers
add_library(test_helpers SHARED
  source_files/test_helpers/fakes_generator.cpp
  source_files/test_helpers/nav2_fakes_generator.cpp)

target_include_directories(test_helpers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_helpers
  rclcpp
  tf2
  tf2_ros
  cv_bridge
  ament_index_cpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  transport_interfaces
  ignition-math6
  nav2_msgs
)

install(TARGETS test_helpers
  EXPORT export_test_helpers
  LIBRARY DESTINATION lib
)

target_link_libraries(test_helpers
  ignition-math6::ignition-math6
)

ament_export_targets(export_test_helpers HAS_LIBRARY_TARGET)
ament_export_libraries(test_helpers)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_include_directories(include)

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
  tf2
  tf2_ros
  cv_bridge
  sensor_msgs
)



ament_package()
