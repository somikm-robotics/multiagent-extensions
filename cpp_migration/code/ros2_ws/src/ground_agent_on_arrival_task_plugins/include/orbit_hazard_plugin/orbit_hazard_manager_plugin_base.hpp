#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/orbit_hazard.hpp"

using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class OrbitHazardManagerPluginBase {
public:
    
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void doHazardOrbit(std::shared_ptr<OrbitHazard::Request> request) = 0;

    virtual ~OrbitHazardManagerPluginBase() = default;

};