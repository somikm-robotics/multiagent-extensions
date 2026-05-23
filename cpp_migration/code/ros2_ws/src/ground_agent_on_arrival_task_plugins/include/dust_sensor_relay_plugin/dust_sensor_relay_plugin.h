#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/msg/dust_sensor_result.hpp"
#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin_base.h"

namespace transportMsgs = transport_interfaces::msg;

class DustSensorRelayPlugin : public DustSensorRelayPluginBase {

private:
    rclcpp::Publisher<transportMsgs::DustSensorResult>::SharedPtr _dustSensorResultPub;
    rclcpp::Subscription<transportMsgs::DustSensorResult>::SharedPtr _dustSensorSub;

    inline static const std::string DUST_SENSOR_TOPIC {"/dust_sensor_reading"};
    rclcpp::Node::SharedPtr _node;

    void dustSensorCallback(const transportMsgs::DustSensorResult& result);
public:
    DustSensorRelayPlugin() = default;

    void Initialise(const rclcpp::Node::SharedPtr & node) override;

    ~DustSensorRelayPlugin() = default;
};