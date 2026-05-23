cmake_minimum_required(VERSION 3.8)
project(ground_agent_nodes)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

#Nodes
# dust_plume_density_estimation_node
add_library(dust_plume_density_estimation_node
  source_files/dust_plume_density_estimation_node.cpp
)

ament_target_dependencies(dust_plume_density_estimation_node  
  rclcpp
  sensor_msgs
  transport_interfaces
)

add_executable(dust_plume_density_estimation_node_exec
  source_files/dust_plume_density_estimation_node.cpp
)

ament_target_dependencies(dust_plume_density_estimation_node_exec
  rclcpp
  sensor_msgs
  transport_interfaces
)

target_link_libraries(dust_plume_density_estimation_node_exec
  dust_plume_density_estimation_node
)

install(TARGETS
  dust_plume_density_estimation_node
  dust_plume_density_estimation_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# mission_handler_node
add_library(mission_handler_node
  source_files/mission_handler_node.cpp
)

ament_target_dependencies(mission_handler_node  
  rclcpp
  geometry_msgs
  transport_interfaces
)

add_executable(mission_handler_node_exec
  source_files/mission_handler_node.cpp
)

ament_target_dependencies(mission_handler_node_exec
  rclcpp
  geometry_msgs
  transport_interfaces
)

target_link_libraries(mission_handler_node_exec
  mission_handler_node
)

install(TARGETS
  mission_handler_node
  mission_handler_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# arrival_task_node
add_library(mission_handler_node
  source_files/arrival_task_node.cpp
)

ament_target_dependencies(arrival_task_node  
  rclcpp
  transport_interfaces
)

add_executable(mission_handler_node_exec
  source_files/mission_handler_node.cpp
)

ament_target_dependencies(mission_handler_node_exec
  rclcpp
  geometry_msgs
  transport_interfaces
)

target_link_libraries(mission_handler_node_exec
  mission_handler_node
)

install(TARGETS
  mission_handler_node
  mission_handler_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

message(STATUS "✅ Ground Agent nodes CMakeLists.txt is running")


ament_package()


