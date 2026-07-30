cmake_minimum_required(VERSION 3.8)
project(supervisory_agent)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shared_infrastructure REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(transport_interfaces REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(
  httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        v0.14.3
)
FetchContent_MakeAvailable(httplib)

include_directories(
  include
)

# alert receiver node
add_library(alert_receiver_node
  source_files/alert_receiver_node.cpp
)

ament_target_dependencies(alert_receiver_node  
  rclcpp
  transport_interfaces
  shared_infrastructure
 
)

add_executable(alert_receiver_node_exec
  source_files/alert_receiver_node_main.cpp
)

ament_target_dependencies(alert_receiver_node_exec
  rclcpp
  transport_interfaces
  shared_infrastructure
)

target_link_libraries(alert_receiver_node_exec
  alert_receiver_node
)

install(TARGETS
  alert_receiver_node
  alert_receiver_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# mission overrrider node
add_library(mission_overrrider_node
  source_files/mission_overrrider_node.cpp
  source_files/supervisory_agent_data_provider.cpp
)

ament_target_dependencies(mission_overrrider_node  
  rclcpp
  transport_interfaces
  shared_infrastructure
)

add_executable(mission_overrrider_node_exec
  source_files/mission_overrrider_node_main.cpp
)

ament_target_dependencies(mission_overrrider_node_exec
  rclcpp
  transport_interfaces
  shared_infrastructure
)

target_link_libraries(mission_overrrider_node_exec
  mission_overrrider_node
)

install(TARGETS
  mission_overrrider_node
  mission_overrrider_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# test alerts node
add_library(test_alerts_node
  source_files/test_alerts_node.cpp
)

ament_target_dependencies(test_alerts_node  
  rclcpp
  transport_interfaces
  shared_infrastructure
)

add_executable(test_alerts_node_exec
  source_files/test_alerts_node.cpp
)

ament_target_dependencies(test_alerts_node_exec
  rclcpp
  transport_interfaces
  shared_infrastructure
)

target_link_libraries(test_alerts_node_exec
  test_alerts_node
)

install(TARGETS
  test_alerts_node
  test_alerts_node_exec
  DESTINATION lib/${PROJECT_NAME}
)


# bridge http server main
add_library(supervisory_agent_server_lib
  source_files/supervisory_agent_bridge_node.cpp
  source_files/supervisory_agent_http_server.cpp
  source_files/supervisory_agent_data_provider.cpp
)

ament_target_dependencies(supervisory_agent_server_lib  
  rclcpp
  transport_interfaces
  shared_infrastructure
)

add_executable(supervisory_agent_web_server_exec
  source_files/supervisory_agent_web_server.cpp
)

ament_target_dependencies(supervisory_agent_web_server_exec
  rclcpp
  transport_interfaces
  shared_infrastructure
  ament_index_cpp
)

target_link_libraries(supervisory_agent_web_server_exec
  supervisory_agent_server_lib
)

install(TARGETS
  supervisory_agent_server_lib
  supervisory_agent_web_server_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install web_ui
# install(DIRECTORY web_ui
#   DESTINATION share/${PROJECT_NAME}
# )

install(DIRECTORY web_ui/
  DESTINATION share/${PROJECT_NAME}/web_ui
)

message(STATUS "✅ Supervisory Agent CMakeLists.txt is running")

ament_package()
