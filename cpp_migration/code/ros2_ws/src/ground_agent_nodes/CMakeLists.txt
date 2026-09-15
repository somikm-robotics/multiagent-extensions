cmake_minimum_required(VERSION 3.8)
project(ground_agent_nodes)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ground_agent_on_arrival_task_plugins REQUIRED)
find_package(ground_agent_mission_handler_plugins REQUIRED)
find_package(ground_agent_dust_plume_actions_plugins REQUIRED)
find_package(shared_infrastructure REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ignition-math6 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(
  include
)

#Nodes
# mission_handler_node
add_library(mission_handler_node
  source_files/mission_handler_node.cpp
)

ament_target_dependencies(mission_handler_node  
  rclcpp
  geometry_msgs
  transport_interfaces
  ground_agent_mission_handler_plugins
  rclcpp_action
  shared_infrastructure
  pluginlib
)

ament_add_gtest(mission_handler_node_test
                  test/mission_handler_node_test.cpp
                  source_files/mission_handler_node.cpp
                  source_files/arrival_task_node.cpp
                  source_files/dust_plume_actions_node.cpp
                  )

ament_target_dependencies(mission_handler_node_test
  rclcpp
  transport_interfaces
  shared_infrastructure
)                  

add_executable(mission_handler_node_exec
  source_files/mission_handler_node_main.cpp
)

ament_target_dependencies(mission_handler_node_exec
  rclcpp
  geometry_msgs
  transport_interfaces
  ground_agent_mission_handler_plugins
  rclcpp_action
  shared_infrastructure
  pluginlib
)

target_link_libraries(mission_handler_node_exec
  mission_handler_node
)

install(TARGETS
  mission_handler_node
  mission_handler_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# dust plume actions node
add_library(dust_plume_actions_node
  source_files/dust_plume_actions_node.cpp
)

ament_target_dependencies(dust_plume_actions_node  
  rclcpp
  sensor_msgs
  transport_interfaces
  ground_agent_dust_plume_actions_plugins
  pluginlib
)

ament_add_gtest(dust_plume_actions_node_test
                  test/dust_plume_actions_node_test.cpp
                  source_files/dust_plume_actions_node.cpp
                  )

ament_target_dependencies(dust_plume_actions_node_test
  rclcpp
  transport_interfaces
  shared_infrastructure
)                  

add_executable(dust_plume_actions_node_exec
  source_files/dust_plume_actions_node_main.cpp
)

ament_target_dependencies(dust_plume_actions_node_exec
  rclcpp
  sensor_msgs
  transport_interfaces
  ground_agent_dust_plume_actions_plugins
  pluginlib
)

target_link_libraries(dust_plume_actions_node_exec
  dust_plume_actions_node
)

install(TARGETS
  dust_plume_actions_node
  dust_plume_actions_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# arrival task node
add_library(arrival_task_node
  source_files/arrival_task_node.cpp
)

ament_target_dependencies(arrival_task_node  
  rclcpp
  transport_interfaces
  shared_infrastructure
  ground_agent_on_arrival_task_plugins
  pluginlib
)

ament_add_gtest(arrival_task_node_test
                  test/arrival_task_node_test.cpp
                  source_files/arrival_task_node.cpp
                  source_files/dust_plume_actions_node.cpp
                  )

ament_target_dependencies(arrival_task_node_test
  rclcpp
  transport_interfaces
  shared_infrastructure
)                  

add_executable(arrival_task_node_exec
  source_files/arrival_task_node_main.cpp
)

ament_target_dependencies(arrival_task_node_exec
  rclcpp
  transport_interfaces
  shared_infrastructure
  ground_agent_on_arrival_task_plugins
  pluginlib
)

target_link_libraries(arrival_task_node_exec
  arrival_task_node
)

install(TARGETS
  arrival_task_node
  arrival_task_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

message(STATUS "✅ Ground Agent nodes CMakeLists.txt is running")


ament_package()


