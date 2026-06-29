#pragma once

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include <chrono>
#include <memory>
#include <atomic>
#include <sstream>

#include "transport_interfaces/msg/start_mission.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/mission_completed.hpp"

#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin_base.hpp"
#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin_base.hpp"


namespace transportMsgs = transport_interfaces::msg;

class DustPlumeActionsNode : public rclcpp::Node {
    
private:
    //Publishers
    rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr missionStatusPub_;

    // subscription
    rclcpp::Subscription<transportMsgs::StartMission>::SharedPtr startMissionSub_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr dustPlumeEstimationCompletedSub_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr dustSensorRelayCompletedSub_;

    // callbackgroups
    rclcpp::CallbackGroup::SharedPtr estimationCallbackGroup_;
    rclcpp::CallbackGroup::SharedPtr sensorRelayCallbackGroup_;

    //synchronisation
    std::atomic_bool started_;
    std::atomic_bool estimationDone_;
    std::atomic_bool sensorRelayDone_;

    // plugins
    pluginlib::ClassLoader<DustSensorRelayPluginBase> dustSensorRelayPluginLoader_;
    std::shared_ptr<DustSensorRelayPluginBase> dustSensorRelayPlugin_;

    pluginlib::ClassLoader<DustPlumeEstimationPluginBase> dustPlumeEstimationPluginLoader_;
    std::shared_ptr<DustPlumeEstimationPluginBase> dustPlumeEstimationPlugin_;

    // variables
    std::ostringstream message_;
    std::optional<bool> missionSuccess_;

    // methods
    void startTasks(const transportMsgs::StartMission startMissionMsg);
    void callDustPlumeEstimation();
    void callDustSensorRelay();
    void notifyOnceDone(std::string message, bool success);
    void dustPlumeEstimationCompletedCallback(transportMsgs::MissionCompleted dustPlumeEstimationCompletedMsg);
    void dustSensorRelayCompletedCallback(transportMsgs::MissionCompleted dustSensorRelayCompletedMsg);

public:
    DustPlumeActionsNode();

    void initialise();

    ~DustPlumeActionsNode() = default;

};