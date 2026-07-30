#pragma once

#include <rclcpp/rclcpp.hpp>

#include "transport_interfaces/msg/hazard_pose.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/base_return.hpp"
#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/msg/dust_sensor_result.hpp"
#include "transport_interfaces/msg/ui_alerts.hpp"


namespace transportMsgs = transport_interfaces::msg;

class AlertReceiverNode : public rclcpp::Node {
public:
    AlertReceiverNode();    

private:
    //publisher
    rclcpp::Publisher<transportMsgs::UIAlerts>::SharedPtr alertsPub_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr notesPub_;

    // subscriptions
    rclcpp::Subscription<transportMsgs::HazardPose>::SharedPtr hazardDetectedSub_;
    rclcpp::Subscription<transportMsgs::MissionStatus>::SharedPtr missionStatusSub_;
    //rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr missionCompletedSub_;
    rclcpp::Subscription<transportMsgs::BaseReturn>::SharedPtr returnToBaseSub_;
    rclcpp::Subscription<transportMsgs::DensityEstimationResult>::SharedPtr densityEstimationResultSub_;
    rclcpp::Subscription<transportMsgs::DustSensorResult>::SharedPtr dustSensorResultSub_;

    void onHazardDetected(const transportMsgs::HazardPose& hazardDetectedMsg);
    void onMissionStatus(const transportMsgs::MissionStatus& missionStatusMsg);
   // void onMissionCompleted(const transportMsgs::MissionCompleted& missionCompletedMsg);
    void onReturnToBase(const transportMsgs::BaseReturn& returnToBaseMsg);
    void onDensityEstimationResult(const transportMsgs::DensityEstimationResult& resultMsg);
    void onDustSensorResult(const transportMsgs::DustSensorResult& result);

    void publishStatus(std::string message, bool enableInspection, bool enableAnalysis, 
                        bool isStatus = false, bool enableScan = false);
};