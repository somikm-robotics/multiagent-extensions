#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>

#include "path_planner_plugin/path_planner_plugin.hpp"

// TODO: implementation later
class AStarPathPlannerPlugin : public PathPlannerPlugin {

public:
    AStarPathPlannerPlugin();

    void plan_path(RequestPathPlan::Goal request,
            SendGoalOptions options) override;

    ~AStarPathPlannerPlugin() override = default;

private:    
};