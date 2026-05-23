cmake_minimum_required(VERSION 3.8)
project(ground_agent_on_arrival_task_plugins)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
  include
)
#Plugins
# dust sensor relay plugin
add_library(dust_sensor_relay_plugin SHARED
  source_files/dust_sensor_relay_plugin.cpp
)

ament_target_dependencies(dust_sensor_relay_plugin
  rclcpp
  transport_interfaces
  pluginlib
)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  include/dust_sensor_relay_plugin/plugin.xml
)

install(TARGETS
  dust_sensor_relay_plugin
  DESTINATION lib/${PROJECT_NAME}
)


message(STATUS "✅ Ground Agent Plugins for ground_agent_on_arrival_task_plugins CMakeLists.txt is running")


ament_package()


