#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <map>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"

#include "orbit_hazard_plugin/orbit_hazard_manager_plugin.hpp"


using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using SetParameters = rcl_interfaces::srv::SetParameters;

using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class Nav2OrbitHazardManagerPlugin : public OrbitHazardManagerPlugin {

private:
    
    //variables
  //  float lastCaptureDegree_;
    bool orbitDone_;

    GoalHandleFollowPath::SharedPtr activeGoalHandleFollowPath_;

    // subscriptions
    rclcpp::Subscription<bool>::SharedPtr orbitDoneSub_;


    // Nav2 client 
    rclcpp::Client<SetParameters>::SharedPtr setParamsClient_;
    rclcpp_action::Client<FollowPath>::SharedPtr followPathActionClient_;
    //rclcpp_action::Client<NavigateToPose>::SharedPtr navigateToPoseActionClient_;


    // future handlers
    void followPathGoalResponse(GoalHandleFollowPath::SharedPtr goal_handle);
    void followPathResult(const GoalHandleFollowPath::WrappedResult& wrappedResult);

    // void navigateToPoseGoalResponse(GoalHandleNavigateToPose::SharedPtr goal_handle);
    // void navigateToPoseResult(const GoalHandleNavigateToPose::WrappedResult& result);

    void setParameterResponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future);
    
    // callbacks
  //  void orbitAngleCallback(const float& msg);
    void orbitDoneCallback(bool msg);
    void cancelDoneCallback(rclcpp_action::Client<FollowPath>::CancelResponse::SharedPtr response);
        
    // methods
    std::optional<nav_msgs::msg::Path> makeMinimalPathInMap();
    void sendFollowPathGoal();
    void cancelFollowPathAndExit();
    
    void setOrbitParams();

    virtual void startOrbit() override;

public:
    Nav2OrbitHazardManagerPlugin();

    virtual void doHazardOrbit(
        std::shared_ptr<OrbitHazard::Request> request) override;

    ~Nav2OrbitHazardManagerPlugin() override = default;
};