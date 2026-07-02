#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/navigate.hpp"

using Navigate = transport_interfaces::srv::Navigate;

class NavigationPluginBase {
public:
    virtual ~NavigationPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void handleNavigation(
        std::shared_ptr<Navigate::Request> request) = 0;

};