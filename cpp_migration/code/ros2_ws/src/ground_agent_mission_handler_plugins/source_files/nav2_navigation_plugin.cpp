#include "navigation_plugin/nav2_navigation_plugin.hpp"
#include "agent_utils.hpp"
#include <stdexcept>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"

Nav2NavigationPlugin::Nav2NavigationPlugin() 
	: logger_{rclcpp::get_logger("Nav2NavigationPlugin")}
{
}


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
	//followWaypointsClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(node, "/follow_waypoints");
	followPathClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(node, "/follow_path");
	serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
	RCLCPP_INFO(node->get_logger(), "Nav2 Navigation Plugin initialised");
}

void Nav2NavigationPlugin::handleNavigation(std::shared_ptr<Navigate::Request> request)
{
	hazardType_ = request->hazard_type;
	try {
		if(!followPathClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(logger_, "Waiting for Nav2 action servers… Timed Out");
			return;
		}

		RCLCPP_INFO(logger_, "Nav2 Navigation handling started");
		nav_msgs::msg::Path path;
		auto node = node_.lock();
		rclcpp::Time now;
		if(!node)
			now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
		else {
			now = node->get_clock()->now();
			RCLCPP_INFO(logger_, "Got node");
		}

		path.header.frame_id = "map";
		path.header.stamp = now;

		path.poses = request->waypoints;
		for(auto &p : path.poses) {
			p.header.stamp = now;
			p.header.frame_id = "map";
		}
		
		// auto goalMsg = FollowWaypoints::Goal();
		// goalMsg.poses = request->waypoints;
		
		auto goalMsg = FollowPath::Goal();
		goalMsg.path = path;
		//RCLCPP_INFO_STREAM(logger_, "Total Path poses: " << goalMsg.poses.size());
		RCLCPP_INFO_STREAM(logger_, "Total Path poses: " << goalMsg.path.poses.size());

		for (auto &w: goalMsg.path.poses) {
			RCLCPP_INFO_STREAM(logger_, "pose x: " << w.pose.position.x << "y: " << w.pose.position.y);
			RCLCPP_INFO_STREAM(logger_, "orientation - w: " << w.pose.orientation.w << "x: " << w.pose.orientation.x);
		}

		// rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
		rclcpp_action::Client<FollowPath>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2NavigationPlugin::followPathGoalResponse, 
													this, std::placeholders::_1);

		//RCLCPP_INFO(logger_, "Sending Nav2 goal");
		followPathClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(logger_, "Exception occured whilst trying to navigate" << ex.what());
	}
}

void Nav2NavigationPlugin::followPathGoalResponse(GoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(logger_, "FollowPath goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "FollowPath goal accepted, waiting for completion");

	followPathClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::followPathResult,
																		this, std::placeholders::_1));
}


void Nav2NavigationPlugin::followPathResult(const GoalHandle::WrappedResult &result)
{
	transportMsgs::NavigationResult navigationResult;
	navigationResult.hazard_type = hazardType_;
	navigationResult.success = result.code == rclcpp_action::ResultCode::SUCCEEDED
								&& result.result->error_code == 0;
	
	if(navigationResult.success) {
		navigationResult.message = "Reached destination successfully!";
		RCLCPP_INFO_STREAM(logger_, "All waypoints reached successfully!");
	} else {
		navigationResult.message = "Error in reaching goal";
		RCLCPP_ERROR(logger_, "Error: Some exception occured in reaching destination - %u", result.result->error_code);
		
	}
	reachedTargetPub_->publish(navigationResult);
	RCLCPP_INFO(logger_, "Navigation result published successfully");
}

/*
void Nav2NavigationPlugin::followWaypointsGoalResponse(GoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(logger_, "FollowWaypoints goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "Waypoints goal accepted, waiting for completion");

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
		RCLCPP_WARN_STREAM(logger_, "Completed with missed indices");
	} else {
		navigationResult.message = "Reached destination successfully!";
		RCLCPP_INFO_STREAM(logger_, "All waypoints reached successfully!");
	}
	reachedTargetPub_->publish(navigationResult);
	RCLCPP_INFO(logger_, "Navigation result published successfully");
}
*/


PLUGINLIB_EXPORT_CLASS(Nav2NavigationPlugin, NavigationPluginBase)