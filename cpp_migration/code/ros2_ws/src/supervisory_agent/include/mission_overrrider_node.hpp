#pragma once

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>

#include "transport_interfaces/msg/ui_alerts.hpp"
#include "transport_interfaces/srv/scan_incident.hpp"
#include "transport_interfaces/srv/override_mission.hpp"
#include "transport_interfaces/msg/hazard_pose.hpp"

#include "shared_infrastructure/data.hpp"

namespace transportMsgs = transport_interfaces::msg;

using ScanIncident = transport_interfaces::srv::ScanIncident;
using OverrideMission = transport_interfaces::srv::OverrideMission;

class MissionOverrideNode : public rclcpp::Node {
public:
    MissionOverrideNode();

private:
    void scanIncident(const ScanIncident::Request::SharedPtr& request, 
                            ScanIncident::Response::SharedPtr response);
      void overrideMission(const OverrideMission::Request::SharedPtr& request, 
        OverrideMission::Response::SharedPtr response);

      //rclcpp::Publisher<transportMsgs::UIAlerts>::SharedPtr alertsPub_;  
      rclcpp::Service<ScanIncident>::SharedPtr scanIncidentUISrv_;
      rclcpp::Service<OverrideMission>::SharedPtr overrideMissionUISrv_;
      rclcpp::Client<OverrideMission>::SharedPtr overrideMissionClient_;
      rclcpp::Publisher<transportMsgs::HazardPose>::SharedPtr hazardDetectedPub_;
      int serviceTimeout_;
};  