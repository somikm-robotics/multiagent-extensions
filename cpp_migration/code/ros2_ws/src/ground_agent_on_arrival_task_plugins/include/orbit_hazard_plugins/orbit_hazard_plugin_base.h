#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/orbit_hazard.hpp"

using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class OrbitHazardPluginBase {
public:
    
    virtual void initialise(const rclcpp::Node::SharedPtr & node);
    virtual void do_hazard_orbit(
        OrbitHazard::Request request,
        OrbitHazard::Response response);

    virtual ~OrbitHazardPluginBase() = default;

};