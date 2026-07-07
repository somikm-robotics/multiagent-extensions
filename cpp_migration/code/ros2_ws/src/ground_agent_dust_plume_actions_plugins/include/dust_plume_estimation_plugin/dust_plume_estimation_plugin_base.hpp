#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/plume_density.hpp"

using PlumeDensity = transport_interfaces::srv::PlumeDensity;

class DustPlumeEstimationPluginBase {
public:
    virtual ~DustPlumeEstimationPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> request) = 0;

};