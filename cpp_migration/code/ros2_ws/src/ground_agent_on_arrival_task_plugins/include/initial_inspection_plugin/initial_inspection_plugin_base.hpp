#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/initial_inspection.hpp"

using InitialInspection = transport_interfaces::srv::InitialInspection;


class InitialInspectionPluginBase {
public:
    virtual ~InitialInspectionPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual std::shared_ptr<InitialInspection::Response> performInspection(
        std::shared_ptr<InitialInspection::Request> request) = 0;

};