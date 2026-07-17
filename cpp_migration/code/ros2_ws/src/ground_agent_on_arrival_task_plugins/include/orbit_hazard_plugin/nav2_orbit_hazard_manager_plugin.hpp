#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <map>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

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

public:
    Nav2OrbitHazardManagerPlugin();

    virtual void initialise(const rclcpp::Node::SharedPtr& node) override;

    virtual void doHazardOrbit(
        std::shared_ptr<OrbitHazard::Request> request) override;

    ~Nav2OrbitHazardManagerPlugin() override = default;    

private:
    
    //variables
    float lastCaptureDegree_;
    bool orbitDone_;

    GoalHandleFollowPath::SharedPtr activeGoalHandleFollowPath_;

    // subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orbitDoneSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr orbitAngleSub_;

    // Nav2 client 
    rclcpp::Client<SetParameters>::SharedPtr setParamsClient_;
    rclcpp_action::Client<FollowPath>::SharedPtr followPathActionClient_;
    //rclcpp_action::Client<NavigateToPose>::SharedPtr navigateToPoseActionClient_;

    // future handlers
    void followPathGoalResponse(GoalHandleFollowPath::SharedPtr goal_handle);
    void followPathResult(const GoalHandleFollowPath::WrappedResult& wrappedResult);

    // commented out - will be removed only after e2e testing
    // void navigateToPoseGoalResponse(GoalHandleNavigateToPose::SharedPtr goal_handle);
    // void navigateToPoseResult(const GoalHandleNavigateToPose::WrappedResult& result);

    void setParameterResponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future);
    
    // callbacks
    void orbitAngleCallback(const std_msgs::msg::Float32& angleMsg);
    void orbitDoneCallback(const std_msgs::msg::Bool& doneMsg);
    void cancelDoneCallback(rclcpp_action::Client<FollowPath>::CancelResponse::SharedPtr response);
        
    // methods
    std::optional<nav_msgs::msg::Path> makeMinimalPathInMap();
    void sendFollowPathGoal();
    void cancelFollowPathAndExit();
    
    void setOrbitParams();

    virtual void startOrbit() override;
};