#pragma once

#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/msg/dust_sensor_result.hpp"
#include "shared_infrastructure/data.hpp"

namespace transportMsgs = transport_interfaces::msg;

class DustSensorRelayPlugin : public DustSensorRelayPluginBase {
public:
    DustSensorRelayPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    void publishDustSensorResult() override;
    
    ~DustSensorRelayPlugin() = default;

private:

    // Publishers
    rclcpp::Publisher<transportMsgs::DustSensorResult>::SharedPtr resultPub_;
    
    // subscription
    rclcpp::Subscription<transportMsgs::DustSensorResult>::SharedPtr resultSub_;
    
    // variables
    rclcpp::Node::SharedPtr node_;
    DustSensorRelayParams configParams_;
    int currentPubCtr_;
    bool startPublish_;

    void sensorCallback(const transportMsgs::DustSensorResult dustSensorResultMsg);
};