cmake_minimum_required(VERSION 3.8)
project(transport_interfaces)

# Interface generator
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OverrideMission.srv"
  "srv/OrbitHazard.srv"
  "srv/Navigate.srv"
  "srv/InitialInspection.srv"
  "srv/PlumeDensity.srv"
  "srv/DensityRegressor.srv"

  "msg/DustSensorResult.msg"
  "msg/HazardPose.msg"
  "msg/NavigationResult.msg"
  "msg/BaseReturn.msg"
  "msg/OnArrivalTask.msg"
  "msg/MissionStatus.msg"
  "msg/MissionCompleted.msg"
  "msg/UIOverrideMission.msg"
  "msg/DensityEstimationResult.msg"
  "msg/StartMission.msg"

  "action/RequestPathPlan.action"

  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Optional: install the .srv files too
install(DIRECTORY srv msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

