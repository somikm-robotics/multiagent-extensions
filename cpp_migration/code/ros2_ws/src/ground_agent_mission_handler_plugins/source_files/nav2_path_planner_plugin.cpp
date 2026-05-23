#include "nav2_path_planner_plugin.h"

void Nav2PathPlannerPlugin::onPathReturn(rclcpp::Client<PathPlanResult>::SharedFuture future)
{
}

Nav2PathPlannerPlugin::onComputePathGoalResponse(rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalFuture future)
{
}

geometry_msgs::msg::PoseStamped Nav2PathPlannerPlugin::getPose(geometry_msgs::msg::Pose goalPose)
{
	return geometry_msgs::msg::PoseStamped();
}

void Nav2PathPlannerPlugin::sendPathPlanResult(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
}

void Nav2PathPlannerPlugin::plan_path(RequestPathPlan::Request request, RequestPathPlan::Response response)
{
}
