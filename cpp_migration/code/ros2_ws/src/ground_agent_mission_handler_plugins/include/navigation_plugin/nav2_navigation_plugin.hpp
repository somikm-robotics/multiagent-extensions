#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/spin.hpp"
#include <memory>

#include "navigation_plugin/navigation_plugin_base.hpp"
#include "transport_interfaces/msg/navigation_result.hpp"
#include "transport_interfaces/msg/mission_completed.hpp"

#include "shared_infrastructure/enums.hpp"

namespace transportMsgs = transport_interfaces::msg;

// using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
// using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

// using FollowPath = nav2_msgs::action::FollowPath;
// using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

using Spin = nav2_msgs::action::Spin;
using SpinGoalHandle = rclcpp_action::ClientGoalHandle<Spin>;


class Nav2NavigationPlugin : public NavigationPluginBase {
public:
    Nav2NavigationPlugin();

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    void handleNavigation(std::shared_ptr<Navigate::Request> request) override;
    void completeRotation(std::shared_ptr<RotateToPosition::Request> request) override;

    ~Nav2NavigationPlugin() = default;

private:
    // publishers
    rclcpp::Publisher<transportMsgs::NavigationResult>::SharedPtr reachedTargetPub_;
    rclcpp::Publisher<transportMsgs::MissionCompleted>::SharedPtr parkingCompletedPub_;

    // Services
    rclcpp::Service<Navigate>::SharedPtr navigateService_;    
    rclcpp::Service<RotateToPosition>::SharedPtr rotateToPositionService_;
    

    // Nav2 client 
    //rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr followWaypointsClient_;
    //rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr followPathClient_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigateThroughPosesClient_;
    rclcpp_action::Client<Spin>::SharedPtr spinClient_;
    // actions
    //rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr computePathToPoseClient_;

    rclcpp::Node::WeakPtr node_;
    rclcpp::Logger logger_;



    // future handlers
    // void followWaypointsGoalResponse(GoalHandle::SharedPtr goal_handle);
    // void followWaypointsResult(const GoalHandle::WrappedResult& result);

    // void followPathGoalResponse(GoalHandle::SharedPtr goal_handle);
    // void followPathResult(const GoalHandle::WrappedResult& result);

    void navigateThroughPosesGoalResponse(NavigateGoalHandle::SharedPtr goal_handle);
    void navigateThroughPosesResult(const NavigateGoalHandle::WrappedResult& result);

    void onSpinResponse(SpinGoalHandle::SharedPtr goal_handle);
    void spinResult(const SpinGoalHandle::WrappedResult& result);

    // variables
    float serviceTimeout_;
    double nav2SpinAllowanceInSecs_;
    std::string hazardType_;
};