# include "orbit_hazard_plugin/nav2_orbit_hazard_manager_plugin.hpp"
#include "agent_utils.hpp"
#include <iomanip>
#include <algorithm>

using Parameter = rcl_interfaces::msg::Parameter;

Nav2OrbitHazardManagerPlugin::Nav2OrbitHazardManagerPlugin()
{
   // lastCaptureDegree_ = 0.0;
    orbitDone_ = false;

    // subscriptions
    // orbitAngleSub_ = node->create_subscription<float>(configParams_.hazardOrbitAngleTopic, 10, [this](float msg) {
    //     this->orbitAngleCallback(msg);
    // });

    orbitDoneSub_ = node_->create_subscription<bool>(configParams_.hazardOrbitDoneTopic, 10, [this](bool msg){
        this->orbitDoneCallback(msg);
    });
    
    // client
    setParamsClient_ = node_->create_client<SetParameters>(configParams_.nav2ControllerServerSetParamsSrv);
    followPathActionClient_ = rclcpp_action::create_client<FollowPath>(node_, "/follow_path");
   // navigateToPoseActionClient_ = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

}

void Nav2OrbitHazardManagerPlugin::doHazardOrbit(std::shared_ptr<OrbitHazard::Request> request)
{
    setFromRequestParams(request);
    navigateToStagingPose(configParams_.nav2PluginOrbitHazardRosbagDir);
    if(stagingCompleted_) 
        this->startOrbit();
}

void Nav2OrbitHazardManagerPlugin::startOrbit()
{
    if(stagingCompleted_) 
        return this->setOrbitParams();
}

void Nav2OrbitHazardManagerPlugin::setOrbitParams()
{
    try {
        if(!setParamsClient_->wait_for_service(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(node_->get_logger(), "Service not available: " << configParams_.nav2ControllerServerSetParamsSrv);
			return;
		}

        auto request = std::make_shared<SetParameters::Request>();
        std::vector<Parameter> params;
        
        Parameter param;
        param.name = "Orbit.center_x";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = hazardCenterPointX_;
        params.push_back(param);

        param.name = "Orbit.center_y";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = hazardCenterPointY_;
        params.push_back(param);

        param.name = "Orbit.radius";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = estimatedHazardDiameter_;
        params.push_back(param);

        param.name = "Orbit.isClockwiseDirection";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        param.value.bool_value = configParams_.isOrbitClockwise;
        params.push_back(param);

        request->parameters = params;
        

        setParamsClient_->async_send_request(request, std::bind
            (&Nav2OrbitHazardManagerPlugin::setParameterResponse, this, std::placeholders::_1));
        

    } catch(const std::runtime_error& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to set params" << ex.what());
    }
}

void Nav2OrbitHazardManagerPlugin::setParameterResponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
{
    auto response = future.get();
    auto results = response->results;

    if(!std::all_of(results.begin(), results.end(),
                        [](const auto& r ) {
                            return r.successful;
                        })) {

        RCLCPP_ERROR(node_->get_logger(), "SetParameters rejected. ");
        
    }
    RCLCPP_INFO(node_->get_logger(), "Orbit parameters set successfully.");
    this->sendFollowPathGoal();
}

void Nav2OrbitHazardManagerPlugin::sendFollowPathGoal()
{
    try {

        if(!followPathActionClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for Nav2 action servers… Timed Out");
			return;
		}

        auto path = this->makeMinimalPathInMap();
        if(!path.has_value()) {
            RCLCPP_WARN(node_->get_logger(), "TF not ready yet. Retrying...");
            return;
        }
        auto goal = FollowPath::Goal();
        goal.path = path.value();
        goal.controller_id = configParams_.nav2OrbitControllerId;
        goal.goal_checker_id = configParams_.nav2GoalCheckerId;
        goal.progress_checker_id = configParams_.nav2ProgressCheckerId;

		rclcpp_action::Client<FollowPath>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2OrbitHazardManagerPlugin::followPathGoalResponse, 
													this, std::placeholders::_1);
		RCLCPP_INFO_STREAM(node_->get_logger(), "Sending FollowPath goal with controller_id = " << configParams_.nav2OrbitControllerId);

		followPathActionClient_->async_send_goal(goal, options);	

    } catch(const std::runtime_error& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to follow path" << ex.what());
    }
}

void Nav2OrbitHazardManagerPlugin::followPathGoalResponse(GoalHandleFollowPath::SharedPtr goal_handle)
{
    if(!goal_handle) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "FollowPAth goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(node_->get_logger(), "Follow Path goal accepted, waiting for completion");

	followPathActionClient_->async_get_result(goal_handle, std::bind(&Nav2OrbitHazardManagerPlugin::followPathResult,
																		this, std::placeholders::_1));
}

void Nav2OrbitHazardManagerPlugin::followPathResult(const GoalHandleFollowPath::WrappedResult &wrappedResult)
{
    try {
        auto result = wrappedResult.result->result;
        auto errorCode = wrappedResult.result->error_code;
        RCLCPP_INFO_STREAM(node_->get_logger(),"FollowPath finished with status: " << errorCode);
        this->shutdownGracefully();

    } catch(const std::runtime_error& ex) {

        RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to get result" << ex.what());
    }
}

void Nav2OrbitHazardManagerPlugin::orbitDoneCallback(bool msg)
{
    if(!msg || orbitDone_)
        return;
    orbitDone_ = true;
    RCLCPP_INFO(node_->get_logger(), "Orbit done received. Cancelling FollowPath to return to normal navigation.");
    this->cancelFollowPathAndExit();
    publishOrbitCompleted(true);
}

void Nav2OrbitHazardManagerPlugin::cancelDoneCallback(rclcpp_action::Client<FollowPath>::CancelResponse::SharedPtr response)
{
    if(response->goals_canceling.size() > 0) {
        RCLCPP_INFO(node_->get_logger(), "FollowPath goal successfully canceled.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "No active goals were canceled.");
    }
    RCLCPP_INFO(node_->get_logger(), "Orbit complete. Exiting Orbiting_Manager.");

    publishOrbitCompleted(false);
    shutdownGracefully();
}

std::optional<nav_msgs::msg::Path> Nav2OrbitHazardManagerPlugin::makeMinimalPathInMap()
{
    auto tfOpt = lookupTransform();
    if(!tfOpt.has_value()) 
        return std::nullopt;

    auto tf = tfOpt.value();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = configParams_.mapFrame;
    pose.header.stamp = node_->get_clock()->now();
    pose.pose.position.x = tf.transform.translation.x + configParams_.hazardMinimalPathOffset * cos(stagingYaw_);
    pose.pose.position.y = tf.transform.translation.y + configParams_.hazardMinimalPathOffset * cos(stagingYaw_);
    pose.pose.position.z = 0.0;

    populateOrientation(pose);

    nav_msgs::msg::Path path;
    path.header = pose.header;
    // Provide at least 2 poses to satisfy validators in some setups
    path.poses.push_back(pose);
    path.poses.push_back(pose);

    return path;
}

void Nav2OrbitHazardManagerPlugin::cancelFollowPathAndExit()
{
    if(!activeGoalHandleFollowPath_) {
        RCLCPP_WARN(node_->get_logger(), "No active FollowPath goal to cancel. Exiting anyway.");
        this->shutdownGracefully();
        return;
    }

    followPathActionClient_->async_cancel_goal(activeGoalHandleFollowPath_, 
                                                std::bind(&Nav2OrbitHazardManagerPlugin::cancelDoneCallback,
                                                          this, std::placeholders::_1 ));
}


/*

void Nav2OrbitHazardManagerPlugin::orbitAngleCallback(const float& msg)
{
    if((msg - lastCaptureDegree_) >= (configParams_.hazardCaptureIntervalDegree - 1e6)) {
        lastCaptureDegree_ = msg;
    }
}

void Nav2OrbitHazardManagerPlugin::navigateToStagingPose()
{
    try {
		if(!navigateToPoseActionClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for Nav2 action servers …");
			return;
		}

		auto goal = NavigateToPose::Goal();
		goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_->get_clock()->now();
        
        auto pose = computeStagingPose();
        if(!pose.has_value()) {
            RCLCPP_WARN(node_->get_logger(), "TF not ready yet. Retrying staging computation...");
            return;
        }

        started_ = true;
        stagingX_ = pose.value().x;
        stagingY_ = pose.value().y;
        stagingYaw_ = pose.value().yaw;

        goal.pose.pose.position.x = stagingX_;
        goal.pose.pose.position.y = stagingY_;
        populateOrientation(goal.pose);

        RCLCPP_INFO_STREAM(node_->get_logger(),"TF ready now...Navigating to Staging pose now - (" << stagingX_ << stagingY_ << " )" );

		rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&Nav2OrbitHazardManagerPlugin::navigateToPoseGoalResponse, 
													this, std::placeholders::_1);
		
		navigateToPoseActionClient_->async_send_goal(goal, options);	

	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to navigate" << ex.what());
	}
}

void Nav2OrbitHazardManagerPlugin::navigateToPoseGoalResponse(GoalHandleNavigateToPose::SharedPtr goal_handle)
{
    if(!goal_handle) {
		RCLCPP_WARN_STREAM(node_->get_logger(), "NavigateToPose goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(node_->get_logger(), "Navigate to pose goal accepted, waiting for completion");

	navigateToPoseActionClient_->async_get_result(goal_handle, std::bind(&Nav2OrbitHazardManagerPlugin::navigateToPoseResult,
																		this, std::placeholders::_1));
}

void Nav2OrbitHazardManagerPlugin::navigateToPoseResult(const GoalHandleNavigateToPose::WrappedResult &result)
{
    try {
        auto errorCode = result.result->error_code;

        // 4 = SUCCEEDED
        if(errorCode == 4) {
            RCLCPP_INFO(node_->get_logger(), "Reached staging pose. Starting orbit.");
            this->setOrbitParams();
            return;
        }

        auto distWithCoords = calculateStagingDistance();
        if(!distWithCoords.has_value()) {
            RCLCPP_WARN(node_->get_logger(), "Navigation failed due to TF issues");
            return;
        }

        auto dist = distWithCoords.value().distance;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << dist;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Distance tp staging: " << ss.str());

        // If navigation failed but robot is close enough
        if (dist < configParams_.hazardNavigatePoseMaxDistance) {
            RCLCPP_INFO(node_->get_logger(), "Navigation failed but close enough to staging. Starting orbit.");
            this->setOrbitParams();
            return;
        }
        RCLCPP_ERROR(node_->get_logger(), "Navigation failed and too far from staging max distance.");
            
    } catch(std::runtime_error& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occured whilst trying to get navigation results" << ex.what());
    }
}



*/

PLUGINLIB_EXPORT_CLASS(Nav2OrbitHazardManagerPlugin, OrbitHazardManagerPluginBase)