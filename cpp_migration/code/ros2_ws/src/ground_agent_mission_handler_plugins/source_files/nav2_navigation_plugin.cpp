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
	parkingCompletedPub_ = node->create_publisher<transportMsgs::MissionCompleted>(
		"completed_rotation", 10);		

	//Service
	navigateService_ = node->create_service<Navigate>(
            "navigate_to_pose", 
                [this] (std::shared_ptr<Navigate::Request> request, 
                    std::shared_ptr<Navigate::Response> response) {
                    this->handleNavigation(request);        
                } );

	rotateToPositionService_ = node->create_service<RotateToPosition>(
            "complete_rotation", 
                [this] (std::shared_ptr<RotateToPosition::Request> request, 
                    std::shared_ptr<RotateToPosition::Response> response) {
                    this->completeRotation(request);        
                } );				
				
	//Nav2 client			
	//followWaypointsClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(node, "/follow_waypoints");
	//followPathClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(node, "/follow_path");
	navigateThroughPosesClient_ = rclcpp_action::create_client<NavigateThroughPoses>(node, "/navigate_through_poses");
	spinClient_ = rclcpp_action::create_client<Spin>(node, "/spin");
	serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
	nav2SpinAllowanceInSecs_ = AgentUtils::getNav2SpinTimeAllowance();
	RCLCPP_INFO(node->get_logger(), "Nav2 Navigation Plugin initialised");
}

void Nav2NavigationPlugin::handleNavigation(std::shared_ptr<Navigate::Request> request)
{
	hazardType_ = request->hazard_type;
	try {
		if(!navigateThroughPosesClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
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
		
		// auto goalMsg = FollowPath::Goal();
		// goalMsg.path = path;

		auto goalMsg = NavigateThroughPoses::Goal();
		goalMsg.poses = request->waypoints;

		RCLCPP_INFO_STREAM(logger_, "Total Path poses: " << goalMsg.poses.size());
		//RCLCPP_INFO_STREAM(logger_, "Total Path poses: " << goalMsg.path.poses.size());

		for (auto &w: goalMsg.poses) {
			RCLCPP_INFO_STREAM(logger_, "pose x: " << w.pose.position.x << "y: " << w.pose.position.y);
			RCLCPP_INFO_STREAM(logger_, "orientation - w: " << w.pose.orientation.w << "x: " << w.pose.orientation.x);
		}

		// rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
		rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2NavigationPlugin::navigateThroughPosesGoalResponse, 
													this, std::placeholders::_1);

		//RCLCPP_INFO(logger_, "Sending Nav2 goal");
		navigateThroughPosesClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(logger_, "Exception occured whilst trying to navigate" << ex.what());
	}
}

void Nav2NavigationPlugin::completeRotation(std::shared_ptr<RotateToPosition::Request> request)
{
	try {
		if(!spinClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(logger_, "Waiting for Nav2 action servers… Timed Out");
			return;
		}

		RCLCPP_INFO(logger_, "Nav2 Spinning to Park started");
		
		// shortest turn, normalised to [-pi, pi]
		double diff = request->target_yaw - request->current_yaw;
		auto targetYaw = std::atan2(std::sin(diff), std::cos(diff)); // fixes unnecessary full circling


		auto goalMsg = Spin::Goal();
		goalMsg.target_yaw = targetYaw;
		goalMsg.time_allowance = rclcpp::Duration::from_seconds(nav2SpinAllowanceInSecs_);

		rclcpp_action::Client<Spin>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2NavigationPlugin::onSpinResponse, 
													this, std::placeholders::_1);

		//RCLCPP_INFO(logger_, "Sending Nav2 goal");
		spinClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(logger_, "Exception occured whilst trying to complete parking" << ex.what());
	}
}

void Nav2NavigationPlugin::onSpinResponse(SpinGoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(logger_, "Spin goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "spin goal accepted, waiting for completion");

	spinClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::spinResult,
																		this, std::placeholders::_1));
}

void Nav2NavigationPlugin::spinResult(const SpinGoalHandle::WrappedResult &result)
{
	transportMsgs::MissionCompleted parkingCompleted;
	parkingCompleted.success = result.code == rclcpp_action::ResultCode::SUCCEEDED
								&& result.result->error_code == 0;
	
	if(parkingCompleted.success) {
		parkingCompleted.message = "Rotation completed successfully!";
		RCLCPP_INFO(logger_, parkingCompleted.message.c_str());
	} else {
		parkingCompleted.message = "Error in parking";
		RCLCPP_ERROR(logger_, "Error: Some exception occured in rotation - %u", result.result->error_code);
		
	}
	parkingCompletedPub_->publish(parkingCompleted);
	RCLCPP_INFO(logger_, "Rotation completed message published successfully");
}

void Nav2NavigationPlugin::navigateThroughPosesGoalResponse(NavigateGoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(logger_, "NavigateThroughPoses goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "NavigateThroughPoses goal accepted, waiting for completion");

	navigateThroughPosesClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::navigateThroughPosesResult,
																		this, std::placeholders::_1));
}

void Nav2NavigationPlugin::navigateThroughPosesResult(const NavigateGoalHandle::WrappedResult &result)
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