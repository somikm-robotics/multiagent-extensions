#pragma once

#include <rclcpp/rclcpp.hpp>
#include "transport_interfaces/msg/dust_sensor_result.hpp"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace transportMsgs = transport_interfaces::msg;
class FakeDustSensorRelayPublisherNode: public rclcpp::Node 
{
public:
    FakeDustSensorRelayPublisherNode();
 
private:

    double getMaximumOpacity(const ignition::math::Vector3d &p, 
                                            const ignition::math::Pose3d &plumePose) const;
    void publishDustSensorResult(); 
    
    rclcpp::Publisher<transportMsgs::DustSensorResult>::SharedPtr dustSensorResultPub_;
    inline static const float scalingConstant_{-0.003};
    rclcpp::TimerBase::SharedPtr pubTimer_;
    double radius_{5.0}, height_{3.0};
    double pm1Max_{220.0}, pm2Max_{550.0}, pm10Max_{900.0};
    double windSpeed_{3.8}, windDirDeg_{120.0}, temperature_{28.0}, humidity_{38.0};
};