#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/navigate.hpp"
#include "transport_interfaces/srv/rotate_to_position.hpp"


using Navigate = transport_interfaces::srv::Navigate;
using RotateToPosition = transport_interfaces::srv::RotateToPosition;

class NavigationPluginBase {
public:
    virtual ~NavigationPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void handleNavigation(
        std::shared_ptr<Navigate::Request> request) = 0;
    virtual void completeRotation(std::shared_ptr<RotateToPosition::Request> request) = 0;    
    
};