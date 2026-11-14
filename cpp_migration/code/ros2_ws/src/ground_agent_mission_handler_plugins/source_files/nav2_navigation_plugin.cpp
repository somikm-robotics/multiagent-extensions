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
	rotationCompletedPub_ = node->create_publisher<transportMsgs::MissionCompleted>(
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
			auto message = "Waiting for Nav2 action servers… Timed Out";
			RCLCPP_INFO_STREAM(logger_, message);
			publishNavigationResult(message, false);
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
		
		auto goalMsg = NavigateThroughPoses::Goal();
		goalMsg.poses = request->waypoints;

		RCLCPP_DEBUG_STREAM(logger_, "Total Path poses: " << goalMsg.poses.size());

		for (auto &w: goalMsg.poses) {
			RCLCPP_DEBUG_STREAM(logger_, "pose x: " << w.pose.position.x << "y: " << w.pose.position.y);
			RCLCPP_DEBUG_STREAM(logger_, "orientation - w: " << w.pose.orientation.w << "x: " << w.pose.orientation.x);
		}

		rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2NavigationPlugin::navigateThroughPosesGoalResponse, 
													this, std::placeholders::_1);

		RCLCPP_DEBUG(logger_, "Sending Nav2 goal");
		navigateThroughPosesClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occured whilst trying to navigate ") + ex.what();
		RCLCPP_ERROR(logger_,  msg.c_str());
		publishNavigationResult(msg, false);
	}
}

void Nav2NavigationPlugin::completeRotation(std::shared_ptr<RotateToPosition::Request> request)
{
	try {
		if(!spinClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			auto message = "Waiting for Nav2 action servers… Timed Out";
			RCLCPP_INFO(logger_, message);
			pubishRotationResult(message, false);
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

		RCLCPP_DEBUG(logger_, "Sending Nav2 goal");
		spinClient_->async_send_goal(goalMsg, options);	

	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occured whilst trying to complete rotation ") + ex.what();
		RCLCPP_ERROR(logger_,  msg.c_str());
		pubishRotationResult(msg, false);
	}
}

void Nav2NavigationPlugin::publishNavigationResult(std::string message, bool success)
{
	transportMsgs::NavigationResult navigationResult;
	navigationResult.hazard_type = hazardType_;
	navigationResult.message = message;
	navigationResult.success = success;

	reachedTargetPub_->publish(navigationResult);
}

void Nav2NavigationPlugin::pubishRotationResult(std::string message, bool success)
{
	transportMsgs::MissionCompleted rotationCompleted;
	rotationCompleted.message = message;
	rotationCompleted.success = success;

	rotationCompletedPub_->publish(rotationCompleted);
}

void Nav2NavigationPlugin::onSpinResponse(SpinGoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		auto message = "Spin goal rejected";
		RCLCPP_WARN_STREAM(logger_, message);
		pubishRotationResult(message, false);
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "spin goal accepted, waiting for completion");

	spinClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::spinResult,
																		this, std::placeholders::_1));
}

void Nav2NavigationPlugin::spinResult(const SpinGoalHandle::WrappedResult &result)
{
	
	bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
						result.result 	&& result.result->error_code == 0;
	
	std::string message;								
	if(success) {
		message = "Rotation completed successfully!";
		RCLCPP_INFO(logger_, message.c_str());
	} else {
		message = "Error in parking";
		RCLCPP_ERROR(logger_, "Error: Some exception occured in rotation - %s", 
					(result.result ? std::to_string(result.result->error_code) : "").c_str());
		
	}
	pubishRotationResult(message, success);
	RCLCPP_INFO(logger_, "Rotation completed message published successfully");
}

void Nav2NavigationPlugin::navigateThroughPosesGoalResponse(NavigateGoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		auto message = "NavigateThroughPoses goal rejected";
		RCLCPP_WARN_STREAM(logger_, message);
		publishNavigationResult(message, false);
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "NavigateThroughPoses goal accepted, waiting for completion");

	navigateThroughPosesClient_->async_get_result(goal_handle, std::bind(&Nav2NavigationPlugin::navigateThroughPosesResult,
																		this, std::placeholders::_1));
}

void Nav2NavigationPlugin::navigateThroughPosesResult(const NavigateGoalHandle::WrappedResult &result)
{
	
	bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
							result.result && result.result->error_code == 0;
	std::string message;
	if(success) {
		message = "Reached destination successfully!";
		RCLCPP_INFO_STREAM(logger_, "All waypoints reached successfully!");
	} else {
		message = "Error in reaching goal";
		RCLCPP_ERROR(logger_, "Error: Some exception occurred in reaching destination - %s", 
						(result.result ? std::to_string(result.result->error_code) : "").c_str());
		
	}
	publishNavigationResult(message, success);

	RCLCPP_INFO(logger_, "Navigation result published successfully");
}

PLUGINLIB_EXPORT_CLASS(Nav2NavigationPlugin, NavigationPluginBase)