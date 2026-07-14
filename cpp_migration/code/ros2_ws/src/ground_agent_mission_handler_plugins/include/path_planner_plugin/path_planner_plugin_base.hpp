#pragma once

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

#include "transport_interfaces/action/request_path_plan.hpp"

using RequestPathPlan = transport_interfaces::action::RequestPathPlan;
using SendGoalOptions = rclcpp_action::Client<RequestPathPlan>::SendGoalOptions;

class PathPlannerPluginBase {
public:
    virtual void initialise(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void plan_path(
        RequestPathPlan::Goal goal, SendGoalOptions options) = 0;

    virtual ~PathPlannerPluginBase() = default;

};