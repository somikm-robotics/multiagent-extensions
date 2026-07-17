#pragma once

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "pluginlib/class_loader.hpp"

#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "navigation_plugin/navigation_plugin_base.hpp"
#include "path_planner_plugin/path_planner_plugin_base.hpp"

#include "transport_interfaces/msg/hazard_pose.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/navigation_result.hpp"
#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/base_return.hpp"

#include "transport_interfaces/srv/override_mission.hpp"
#include "transport_interfaces/srv/navigate_request.hpp"
#include "transport_interfaces/srv/navigate.hpp"

#include "transport_interfaces/action/request_path_plan.hpp"

#include "shared_infrastructure/enums.hpp"
#include "agent_utils.hpp"

namespace transportMsgs = transport_interfaces::msg;
using OverrideMission = transport_interfaces::srv::OverrideMission;
using Navigate = transport_interfaces::srv::Navigate;

using RequestPathPlan = transport_interfaces::action::RequestPathPlan;
using GoalHandleRequestPathPlan = rclcpp_action::ClientGoalHandle<RequestPathPlan>;


class MissionHandlerNode : public rclcpp::Node {
public:
    MissionHandlerNode();
    
    void initialise();
    
    ~MissionHandlerNode() = default;
    
private:
    //publishers
    rclcpp::Publisher<transportMsgs::OnArrivalTask>
            ::SharedPtr onArrivalTaskStartPub_;
    rclcpp::Publisher<transportMsgs::BaseReturn>::SharedPtr baseReturnPub_;
    rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr intermediadyMissionStatusPub_;

    // subscriptions
    rclcpp::Subscription<transportMsgs::HazardPose>::SharedPtr hazardDetectedSub_;
    rclcpp::Subscription<transportMsgs::NavigationResult>::SharedPtr navigationResultSub_;
    rclcpp::Subscription<transportMsgs::MissionStatus>::SharedPtr onArrivalTaskCompletionSub_;

    //services
    rclcpp::Service<OverrideMission>::SharedPtr overrideMissionService_;
    
    // subscription callbacks
    void hazardDetectedCallback(const transportMsgs::HazardPose& hazardPoseMessage);
    void reachedTargetCallback(const transportMsgs::NavigationResult& resultMessage);
    void onArrivalTaskCompletion(transportMsgs::MissionStatus arrivalTaskCompletionMessage);

    // service methods
    void handleOverride(std::shared_ptr<OverrideMission::Request> request, 
        std::shared_ptr<OverrideMission::Response> response);
    
    // methods
    void checkOverrideStatus();
    void publishIntermediaryStatus(std::string& message, const bool toggleCommand);
    geometry_msgs::msg::Pose getReturnPose();
    void setOrbitGoal();
    void requestPathForNavigation();
    void sendNavigationCommand(std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
    void waitForApprovalOrTimeout();
    void publishOnArrivalTask();
    void setNextTaskType();
    void startReturnToBase(transportMsgs::MissionStatus missionStatusMessage);
    void publishReturnToBaseStatus();
    Edge computeEdge(float robotPosX, float robotPosY, float hazardCenterPosX, float hazardCenterPosY);


    // request path plan action methods
    void requestPathPlan();
    void handlePathPlanResult(const GoalHandleRequestPathPlan::WrappedResult& result);

    // plugins
    pluginlib::ClassLoader<NavigationPluginBase> navigationPluginLoader_;
    std::shared_ptr<NavigationPluginBase> navigationPlugin_;

    pluginlib::ClassLoader<PathPlannerPluginBase> pathPlannerPluginLoader_;
    std::shared_ptr<PathPlannerPluginBase> pathPlannerPlugin_;

    // variables
    std::optional<bool> approved_;
    std::optional<bool> orbitApproved_;
    bool hazardReceived_;
    HazardType hazardType_;
    std::string hazardTypeStr_;
    TaskType taskType_;
    geometry_msgs::msg::Pose hazardPose_;
    bool isAtBase_;
    float estimatedHazardDiameter_;
    geometry_msgs::msg::Point hazardCenterPoint_;
    SimplePose agilexPose_;
    float serviceTimeout_;
    int navigationOverrideTimeoutInMinutes_;
    bool tasksDone_;
    bool isReturn_;
    rclcpp::TimerBase::SharedPtr overrideTimer_;
    rclcpp::TimerBase::SharedPtr overrideTimerStep2_;
    geometry_msgs::msg::Pose hazardOrbitPose_;
    geometry_msgs::msg::Pose returnPose_;

};