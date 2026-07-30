#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <string>


#include "transport_interfaces/msg/ui_alerts.hpp"
#include "transport_interfaces/srv/scan_incident.hpp"
#include "transport_interfaces/srv/override_mission.hpp"

namespace transportMsgs = transport_interfaces::msg;
using ScanIncident = transport_interfaces::srv::ScanIncident;
using OverrideMission = transport_interfaces::srv::OverrideMission;

class SupervisoryAgentBridgeNode : public rclcpp::Node {
public:

    SupervisoryAgentBridgeNode();

    // Blocking — waits until a status message is available, returns it
    //std::string waitForMessage(bool isStatus);
    std::string waitForStatusMessage();
    std::string waitForHistoryMessage();

    // service calls
    std::pair<bool, std::string> overrideMission(const std::string& label, const std::string& action);
    std::pair<bool, std::string> scanIncident(const std::string& locationName);

    // get method to populate locations
    std::string getLocationsJson();

private:
    void onAlert(transportMsgs::UIAlerts alertsMsg);

    int serviceTimeout_;
    rclcpp::Subscription<transportMsgs::UIAlerts>::SharedPtr statusSub_;
    rclcpp::Client<ScanIncident>::SharedPtr scanIncidentClient_;
    rclcpp::Client<OverrideMission>::SharedPtr overrideMissionClient_;

    std::deque<std::string> statusMessageQueue_;
    std::mutex statusQueueMutex_;
    std::condition_variable statusQueueCv_;

    std::deque<std::string> historyMessageQueue_;
    std::mutex historyQueueMutex_;
    std::condition_variable historyQueueCv_;

};