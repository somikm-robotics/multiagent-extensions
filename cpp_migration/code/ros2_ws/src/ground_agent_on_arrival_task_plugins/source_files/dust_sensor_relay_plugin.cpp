#include "pluginlib/class_list_macros.hpp"
#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin.h"

void DustSensorRelayPlugin::Initialise(const rclcpp::Node::SharedPtr & node) {

    _node = node;
    _dustSensorResultPub = _node->create_publisher<transportMsgs::DustSensorResult>(
        "/dust_sensor_reading", 10);
    
    _dustSensorSub = _node->create_subscription<transportMsgs::DustSensorResult>(
    DUST_SENSOR_TOPIC, 10, [this](const transportMsgs::DustSensorResult& result){
        this->dustSensorCallback(result);
    });
}

void DustSensorRelayPlugin::dustSensorCallback(const transportMsgs::DustSensorResult &result) {
    
};

PLUGINLIB_EXPORT_CLASS(DustSensorRelayPlugin, DustSensorRelayPluginBase)

