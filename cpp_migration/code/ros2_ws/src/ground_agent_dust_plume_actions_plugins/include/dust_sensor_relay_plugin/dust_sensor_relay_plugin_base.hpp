#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>


class DustSensorRelayPluginBase {
public:
    virtual ~DustSensorRelayPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void publishDustSensorResult() = 0;
};