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
#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/msg/dust_sensor_result.hpp"

#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin_base.hpp"
#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin_base.hpp"


namespace transportMsgs = transport_interfaces::msg;

class DustPlumeActionsNode : public rclcpp::Node {
    
private:
    //Publishers
    rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr missionStatusPub_;

    // subscription
    rclcpp::Subscription<transportMsgs::StartMission>::SharedPtr startMissionSub_;
    rclcpp::Subscription<transportMsgs::DensityEstimationResult>::SharedPtr estimationCompletedSub_;
    rclcpp::Subscription<transportMsgs::DustSensorResult>::SharedPtr sensorRelayCompletedSub_;

    // callbackgroups
    rclcpp::CallbackGroup::SharedPtr estimationCallbackGroup_;
    rclcpp::CallbackGroup::SharedPtr sensorRelayCallbackGroup_;

    //synchronisation
    std::atomic_bool started_;
    std::atomic_bool estimationDone_;
    std::atomic_bool sensorRelayDone_;

    // timers
    rclcpp::TimerBase::SharedPtr estimationTimer_;
    rclcpp::TimerBase::SharedPtr sensorRelayTimer_;

    // plugins
    pluginlib::ClassLoader<DustSensorRelayPluginBase> dustSensorRelayPluginLoader_;
    pluginlib::ClassLoader<DustPlumeEstimationPluginBase> dustPlumeEstimationPluginLoader_;
    
    std::shared_ptr<DustSensorRelayPluginBase> dustSensorRelayPlugin_;
    std::shared_ptr<DustPlumeEstimationPluginBase> dustPlumeEstimationPlugin_;

    // variables
    std::ostringstream message_;
    std::optional<bool> missionSuccess_;

    // methods
    void callDustPlumeEstimation();
    void callDustSensorRelay();
    void notifyOnceDone(std::string message, bool success);
    void dustPlumeEstimationCompletedCallback(const transportMsgs::DensityEstimationResult&densityEstimationResultMsg);
    void dustSensorRelayCompletedCallback(const transportMsgs::DustSensorResult& dustSensorResultMsg);

public:
    DustPlumeActionsNode();

    void initialise();
    void startTasks(const transportMsgs::StartMission startMissionMsg);

    ~DustPlumeActionsNode();

};