cmake_minimum_required(VERSION 3.8)
project(indoor_ground_agent)

# Enable Python-based installation
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

# Install the shell wrapper for ROS2 CLI to find
install(PROGRAMS
  scripts/twist_relay_node
  scripts/orbit_generator
  scripts/orbit_manager

  DESTINATION lib/${PROJECT_NAME}
)

# C++ plug ins
add_library(orbit_controller SHARED
  plugins/orbit_controller.cpp
)

target_include_directories(orbit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orbit_controller
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

target_link_libraries(orbit_controller
  pluginlib::pluginlib
)

# Recommended for pluginlib
target_compile_definitions(orbit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS orbit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)


# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# urdfs
install(DIRECTORY urdfs
  DESTINATION share/${PROJECT_NAME}/
)

# visualisations
install(DIRECTORY visualisations
  DESTINATION share/${PROJECT_NAME}/
)

# maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

message(STATUS "✅ Indoor Ground Agent CMakeLists.txt is running")


ament_export_dependencies(rosidl_default_runtime)

ament_export_libraries(orbit_controller)
ament_export_include_directories(include)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_package()

