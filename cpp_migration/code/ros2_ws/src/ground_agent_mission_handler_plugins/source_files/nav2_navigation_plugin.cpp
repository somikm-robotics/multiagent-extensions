#include "navigation_plugin/nav2_navigation_plugin.hpp"
#include "agent_utils.hpp"
#include <stdexcept>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"

void Nav2NavigationPlugin::initialise(const rclcpp::Node::SharedPtr &node) 
{
	node_ = node;
	// Publisher
	reachedTargetPub_ = node->create_publisher<transportMsgs::NavigationResult>(
        "reached_target", 10);

	//Service
	navigateService_ = node->create_service<Navigate>(
            "navigate_to_pose", 
                [this] (std::shared_ptr<Navigate::Request> request, 
                    std::shared_ptr<Navigate::Response> response) {
                    this->handleNavigation(request);        
                } );
				
	//Nav2 client			
	followWaypointsClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(node, "/follow_waypoints");
	serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
	RCLCPP_INFO(node->get_logger(), "Nav2 Navigation Plugin initialised");
}

void Nav2NavigationPlugin::handleNavigation(std::shared_ptr<Navigate::Request> request)
{
	hazardType_ = request->hazard_type;
	try {
		if(!followWaypointsClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for Nav2 action servers… Timed Out");
			return;
		}

		//RCLCPP_INFO(node_->get_logger(), "Nav2 Navigation handling started");
		auto goalMsg = FollowWaypoints::Goal();
		goalMsg.poses = request->waypoints;

		rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2NavigationPlugin::followWaypointsGoalResponse, 
													this, std::placeholders::_1);

		//RCLCPP_INFO(node_->get_logger(), "Sending Nav2 goal");
		followWaypointsClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to navigate" << ex.what());
	}
}

void Nav2NavigationPlugin::followWaypointsGoalResponse(GoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "FollowWaypoints goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(node_->get_logger(), "Waypoints goal accepted, waiting for completion");

	followWaypointsClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::followWaypointsResult,
																		this, std::placeholders::_1));
}

void Nav2NavigationPlugin::followWaypointsResult(const GoalHandle::WrappedResult &result)
{
	transportMsgs::NavigationResult navigationResult;
	navigationResult.hazard_type = hazardType_;
	navigationResult.success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
	if(!result.result->missed_waypoints.empty()) {
		navigationResult.message = "Reached destination but missed a few waypoints";
		RCLCPP_WARN_STREAM(node_->get_logger(), "Completed with missed indices");
	} else {
		navigationResult.message = "Reached destination successfully!";
		RCLCPP_INFO_STREAM(node_->get_logger(), "All waypoints reached successfully!");
	}
	reachedTargetPub_->publish(navigationResult);
	RCLCPP_INFO(node_->get_logger(), "Navigation result published successfully");
}
PLUGINLIB_EXPORT_CLASS(Nav2NavigationPlugin, NavigationPluginBase)