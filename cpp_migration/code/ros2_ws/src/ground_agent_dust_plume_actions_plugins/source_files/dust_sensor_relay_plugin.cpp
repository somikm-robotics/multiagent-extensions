#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin.hpp"

#include "pluginlib/class_list_macros.hpp"
#include <array>

#include "agent_utils.hpp"

DustSensorRelayPlugin::DustSensorRelayPlugin() 
            : logger_(rclcpp::get_logger("DustSensorRelayPlugin"))
{
}

void DustSensorRelayPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
    startPublish_ = false;
    node_ = node;
    //logger_ = node->get_logger();
    resultPub_ = node->create_publisher<transportMsgs::DustSensorResult>(
                        "dust_sensor_reading", 10);

    resultSub_ =   node->create_subscription<transportMsgs::DustSensorResult>("/dust/agilex_diff_drive", 10,
        [this](const transportMsgs::DustSensorResult dustSensorResultMsg){
        this->sensorCallback(dustSensorResultMsg);
    });

    currentPubCtr_ = 0;
    configParams_ = AgentUtils::getDustSensorRelayParams();
}

void DustSensorRelayPlugin::publishDustSensorResult()
{
    startPublish_ = true;
}

void DustSensorRelayPlugin::sensorCallback(transportMsgs::DustSensorResult dustSensorResultMsg)
{
    if(!startPublish_ || currentPubCtr_ > configParams_.MaxPublishCount)
        return;
    
   std::array<float, 3> pmValues { dustSensorResultMsg.pm1_0, dustSensorResultMsg.pm2_5, dustSensorResultMsg.pm10};
   auto availableCount = std::count_if(pmValues.begin(), 
                                        pmValues.end(),
                                        [this](const auto& val) {
                                            return val > this->configParams_.PublishThreshold;  
                                        });
                
    if(availableCount >= configParams_.MinValuesAvailibity  ) {
        dustSensorResultMsg.success = true;
        resultPub_->publish(dustSensorResultMsg);
        currentPubCtr_++;
        RCLCPP_INFO(logger_, "Dust detected – published reading");
        RCLCPP_INFO_STREAM(
            logger_,
            std::fixed
            << "PM1.0=" << std::setprecision(1) << dustSensorResultMsg.pm1_0 << " µg/m³, "
            << "PM2.5=" << std::setprecision(1) << dustSensorResultMsg.pm2_5 << " µg/m³, "
            << "PM10=" << std::setprecision(1) << dustSensorResultMsg.pm10 << " µg/m³, "
            << "TSP=" << std::setprecision(1) << dustSensorResultMsg.tsp << " µg/m³, "
            << "Opacity=" << std::setprecision(2) << dustSensorResultMsg.opacity << ", "
            << "Wind=" << std::setprecision(1) << dustSensorResultMsg.wind_speed << " m/s @ "
            << std::setprecision(0) << dustSensorResultMsg.wind_direction << "°, "
            << "RH=" << std::setprecision(0) << dustSensorResultMsg.humidity << " %, "
            << "Temp=" << std::setprecision(1) << dustSensorResultMsg.temperature << " °C");
    }
}

PLUGINLIB_EXPORT_CLASS(DustSensorRelayPlugin, DustSensorRelayPluginBase)