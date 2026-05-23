#include "rclcpp/rclcpp.hpp"
#include <memory>


class DustSensorRelayPluginBase {
public:
    virtual ~DustSensorRelayPluginBase() = default;
    virtual void Initialise(const rclcpp::Node::SharedPtr & node);

};