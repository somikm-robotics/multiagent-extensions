#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>

#include "path_planner_plugin/path_planner_plugin.hpp"

class AStarPathPlannerPlugin : public PathPlannerPlugin {


private:

public:
    AStarPathPlannerPlugin();

    void plan_path(RequestPathPlan::Goal request,
            SendGoalOptions options) override;

    ~AStarPathPlannerPlugin() override = default;
};