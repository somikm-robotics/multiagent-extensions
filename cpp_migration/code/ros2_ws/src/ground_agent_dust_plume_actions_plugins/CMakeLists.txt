cmake_minimum_required(VERSION 3.8)
project(ground_agent_dust_plume_actions_plugins)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_infrastructure REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
)

# dust sensor relay plugin
add_library(dust_sensor_relay_plugin SHARED
  source_files/dust_sensor_relay_plugin.cpp
)

target_include_directories(dust_sensor_relay_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dust_sensor_relay_plugin
  rclcpp
  transport_interfaces
  pluginlib
  shared_infrastructure
)

ament_add_gtest(dust_sensor_relay_plugin_test
  test/dust_sensor_relay_plugin_test.cpp
)

ament_target_dependencies(dust_sensor_relay_plugin_test
  rclcpp
  transport_interfaces
  pluginlib
)

target_link_libraries(dust_sensor_relay_plugin_test
                      dust_sensor_relay_plugin
                      ignition-math6::ignition-math6)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/dust_sensor_relay_plugin/plugin.xml
)

install(TARGETS
  dust_sensor_relay_plugin
  EXPORT export_dust_sensor_relay_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(dust_sensor_relay_plugin)
ament_export_targets(export_dust_sensor_relay_plugin HAS_LIBRARY_TARGET)

# dust plume estimation plugin
add_library(dust_plume_estimation_plugin SHARED
  source_files/dust_plume_estimation_plugin.cpp
)

target_include_directories(dust_plume_estimation_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dust_plume_estimation_plugin
  rclcpp
  transport_interfaces
  pluginlib
  sensor_msgs
  shared_infrastructure
  cv_bridge
)

ament_add_gtest(dust_plume_estimation_plugin_test
  test/dust_plume_estimation_plugin_test.cpp
)

ament_target_dependencies(dust_plume_estimation_plugin_test
  rclcpp
  transport_interfaces
  pluginlib
  sensor_msgs
  shared_infrastructure
  cv_bridge
  ament_index_cpp
)

target_link_libraries(dust_plume_estimation_plugin_test
                      dust_plume_estimation_plugin
                      ${OpenCV_LIBS}
                      Boost::boost)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/dust_plume_estimation_plugin/plugin.xml
)

install(TARGETS
  dust_plume_estimation_plugin
  EXPORT export_dust_plume_estimation_plugin
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(dust_plume_estimation_plugin)
ament_export_targets(export_dust_plume_estimation_plugin HAS_LIBRARY_TARGET)

install(DIRECTORY include/
  DESTINATION include
)


message(STATUS "✅ Ground Agent - ground_agent_dust_plume_action_plugins CMakeLists.txt is running")


ament_package()


