cmake_minimum_required(VERSION 3.8)
project(multi_agent_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/python3.10/site-packages
)

# twist relay node
add_library(twist_relay_node
  source_files/twist_relay_node.cpp
)

ament_target_dependencies(twist_relay_node
  rclcpp
  geometry_msgs
)

add_executable(twist_relay_node_exec
  source_files/twist_relay_node.cpp
)

ament_target_dependencies(twist_relay_node_exec
  rclcpp
  sensor_msgs
)

target_link_libraries(twist_relay_node_exec
  twist_relay_node
)

install(TARGETS
  twist_relay_node
  twist_relay_node_exec
  DESTINATION lib/${PROJECT_NAME}
)


# gps covariance injector node
add_library(gps_covariance_injector_node
  source_files/gps_covariance_injector_node.cpp
)

ament_target_dependencies(gps_covariance_injector_node
  rclcpp
  sensor_msgs
)

add_executable(gps_covariance_injector_node_exec
  source_files/gps_covariance_injector_node.cpp
)

ament_target_dependencies(gps_covariance_injector_node_exec
  rclcpp
  sensor_msgs
)

target_link_libraries(gps_covariance_injector_node_exec
  gps_covariance_injector_node
)

install(TARGETS
  gps_covariance_injector_node
  gps_covariance_injector_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# imu covariance injector node
add_library(imu_covariance_injector_node
  source_files/imu_covariance_injector_node.cpp
)

ament_target_dependencies(imu_covariance_injector_node
  rclcpp
  sensor_msgs
)

add_executable(imu_covariance_injector_node_exec
  source_files/imu_covariance_injector_node.cpp
)

ament_target_dependencies(imu_covariance_injector_node_exec
  rclcpp
  sensor_msgs
)

target_link_libraries(imu_covariance_injector_node_exec
  imu_covariance_injector_node
)

install(TARGETS
  imu_covariance_injector_node
  imu_covariance_injector_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    worlds
    models
    urdf
    rviz
    maps
    config
  DESTINATION
    share/${PROJECT_NAME}

)

ament_package()