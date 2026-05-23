#include "nav2_navigation_plugin.h"

Nav2NavigationPlugin::followPathGoalResponse(
	rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalFuture future)
{
}

Nav2NavigationPlugin::followPathResult(rclcpp_action::Client<nav2_msgs::action::FollowPath>::ResultFuture future)
{
}

void Nav2NavigationPlugin::initialise(const rclcpp::Node::SharedPtr& node)
{
}

void Nav2NavigationPlugin::handleNavigation(NavigateRequest::Request request, NavigateRequest::Response response)
{
}
