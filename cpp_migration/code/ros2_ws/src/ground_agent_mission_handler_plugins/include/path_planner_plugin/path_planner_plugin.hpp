#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>


#include "path_planner_plugin/path_planner_plugin_base.hpp"


class PathPlannerPlugin : public PathPlannerPluginBase {

private:
   

protected:
    // actions
    rclcpp_action::Client<RequestPathPlan>::SharedPtr requestPathPlanActionClient_;

    rclcpp::Node::SharedPtr node_;
    float serviceTimeout_;
    SendGoalOptions requestOptions_;

    void coreInitialise(const rclcpp::Node::SharedPtr& node);

public:
    PathPlannerPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override = 0;

    virtual void plan_path(
        RequestPathPlan::Goal request, SendGoalOptions options) override = 0;

    ~PathPlannerPlugin() override = default;
};