#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"

using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class Nav2CommanderOrbitHazardPlugin : public OrbitHazardPlugin {
//TODO: in Phase 2
private:
    
public:
    Nav2CommanderOrbitHazardPlugin()=default;

    ~Nav2CommanderOrbitHazardPlugin() = default;
};