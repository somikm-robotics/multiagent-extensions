cmake_minimum_required(VERSION 3.8)
project(ground_agent_dust_plume_density_estimation_plugins)

# Enable C++-based installation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
  include
)


message(STATUS "✅ Ground Agent - ground_agent_dust_plume_density_estimation_plugins CMakeLists.txt is running")


ament_package()


