#include "mission_handler_node.hpp"
#include <stdexcept>
#include <functional>
#include <iostream>
#include <chrono>
#include <cmath>
#include <tf2/utils.h>
#include "agent_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using RequestPathPlan = transport_interfaces::action::RequestPathPlan;
using GoalHandle = rclcpp_action::ClientGoalHandle<RequestPathPlan>;
using SendGoalOptions = rclcpp_action::Client<RequestPathPlan>::SendGoalOptions;

MissionHandlerNode::MissionHandlerNode()  : Node ("mission_handler_node"),
											navigationPluginLoader_("ground_agent_mission_handler_plugins", "NavigationPluginBase"),
											pathPlannerPluginLoader_("ground_agent_mission_handler_plugins", "PathPlannerPluginBase")
{
	//Publishers
	onArrivalTaskStartPub_ = this->create_publisher<transportMsgs::OnArrivalTask>(
        "on_arrival_task", 10);
	baseReturnPub_ = this->create_publisher<transportMsgs::BaseReturn>(
        "return_to_base_status", 10);
	intermediadyMissionStatusPub_ = this->create_publisher<transportMsgs::MissionStatus>(
        "intermediary_mission_status", 10);

	//Subscribers
	hazardDetectedSub_ = this->create_subscription<transportMsgs::HazardPose>(
    "hazard_detected", 10, [this](const transportMsgs::HazardPose& result){
        this->hazardDetectedCallback(result);
    });
	navigationResultSub_ = this->create_subscription<transportMsgs::NavigationResult>(
    "reached_target", 10, [this](const transportMsgs::NavigationResult& result){
        this->reachedTargetCallback(result);
    });
	onArrivalTaskCompletionSub_ = this->create_subscription<transportMsgs::MissionStatus>(
    "on_arrival_task_mission_status", 10, [this](const transportMsgs::MissionStatus& result){
        this->onArrivalTaskCompletion(result);
    });

	parkingCompletedSub_ = this->create_subscription<transportMsgs::MissionCompleted>("completed_rotation", 10,
							[this](const transportMsgs::MissionCompleted& parkingCompletedMessage) {
								this->completedParking(parkingCompletedMessage);
							});

	//Services
	overrideMissionService_ = this->create_service<OverrideMission>(
            "override_mission", 
                [this] (std::shared_ptr<OverrideMission::Request> request, 
                    std::shared_ptr<OverrideMission::Response> response) {
                    this->handleOverride(request, response
                    );        
                } );


	
	// plugin
	navigationPlugin_ = navigationPluginLoader_.createSharedInstance("Nav2NavigationPlugin");
	pathPlannerPlugin_ = pathPlannerPluginLoader_.createSharedInstance("Nav2PathPlannerPlugin");

	// variables
	//tf
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);
	
	missionStatus_ = MissionStatus::READY;
	hazardType_ = HazardType::NONE;
	taskType_ = TaskType::NONE;
	estimatedHazardDiameter_ = 0.0;
	serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
	navigationOverrideTimeoutInMinutes_ = AgentUtils::getNavigationOverrideTimeoutFromConfig();
	tasksDone_ = false;
	isReturn_ = false;
}

void MissionHandlerNode::initialise()
{
	navigationPlugin_->initialise(this->shared_from_this());
	pathPlannerPlugin_->initialise(this->shared_from_this());
}

void MissionHandlerNode::hazardDetectedCallback(const transportMsgs::HazardPose& hazardPoseMessage)
{
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Ground Agent received " << hazardPoseMessage.hazard_type);
	RCLCPP_DEBUG(this->get_logger(), "Mission status: %d", static_cast<int>(missionStatus_));

	if(missionStatus_ != MissionStatus::READY && missionStatus_ != MissionStatus::UNKNOWN) {
		std::string msg = "Hazard Processing in progress";
		RCLCPP_INFO(this->get_logger(), msg.c_str());
		publishIntermediaryStatus(msg, false);
		return;
	}

	if(missionStatus_ == MissionStatus::UNKNOWN) {
		RCLCPP_WARN(get_logger(), "Existing previous error! New Hazard received, processing...");
	}

	missionStatus_ = MissionStatus::STARTED;

	RCLCPP_DEBUG_STREAM(this->get_logger(), "hazard center point: x: " << hazardPoseMessage.hazard_center_point.x << " y: " << hazardPoseMessage.hazard_center_point.y );
	
	RCLCPP_DEBUG_STREAM(this->get_logger(), "hazard pose: x: " << hazardPoseMessage.pose.position.x << " y: " << hazardPoseMessage.pose.position.y );

	hazardTypeStr_ = hazardPoseMessage.hazard_type;
	hazardType_ = AgentUtils::parseHazardType(hazardPoseMessage.hazard_type);
	if(hazardType_ == HazardType::NONE) {
		auto msg = std::string("Invalid Hazard type- ") + hazardTypeStr_;
		RCLCPP_ERROR_STREAM(this->get_logger(), msg);
		missionStatus_ = MissionStatus::UNKNOWN;
		publishIntermediaryStatus(msg, false);
		return;
	}

	taskType_ = hazardType_ == HazardType::FIBROUS_HAZARD ? TaskType::INITIAL_INSPECTION 
														: TaskType::DUST_PLUME_ACTIONS;
								
	hazardPose_ = hazardPoseMessage.pose;
	estimatedHazardDiameter_ = hazardPoseMessage.estimated_hazard_diameter;
	hazardCenterPoint_ = hazardPoseMessage.hazard_center_point;
	agilexPose_ = AgentUtils::getAgilexBasePose();

	try
	{
		auto msg = "Waiting for " + std::to_string(navigationOverrideTimeoutInMinutes_) + " minute(s)";
		RCLCPP_INFO(this->get_logger(), msg.c_str());
		overrideTimer_ = this->create_wall_timer(std::chrono::minutes(navigationOverrideTimeoutInMinutes_),
													std::bind(&MissionHandlerNode::checkOverrideStatus, this));
		RCLCPP_DEBUG(this->get_logger(), "Created wall timer");
		publishIntermediaryStatus(msg, true);
	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occurred - ") + ex.what();
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		publishIntermediaryStatus(msg, false);
		missionStatus_ = MissionStatus::UNKNOWN;
	}
}

void MissionHandlerNode::handleOverride(std::shared_ptr<OverrideMission::Request> request, std::shared_ptr<OverrideMission::Response> response)
{
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Ground Agent received " << request->command);

	if(missionStatus_ != MissionStatus::STARTED && missionStatus_ != MissionStatus::PENDINGANALYSIS) {
		RCLCPP_WARN(get_logger(), "Mission is either not started or already in progress");
		return;
	}

	auto commandType = AgentUtils::parseCommandType(request->command);

	std::string msg = commandType == CommandType::APPROVED ? "Approved! Dispatching mission."
													: "Cancelled! Supervisor aborted the mission";
	if(commandType == CommandType::APPROVED) {
		response->status = "Dispatched";
		response->success = true;
		this->publishIntermediaryStatus(msg, true);
		RCLCPP_INFO_STREAM(this->get_logger(), msg);
	
		if(taskType_ == TaskType::INITIAL_INSPECTION
		|| taskType_ == TaskType::DUST_PLUME_ACTIONS) {

			missionStatus_ = MissionStatus::APPPROVEDINITIAL;
			
			this->requestPathForNavigation();
		}
		else if(taskType_ == TaskType::ASBESTOS_ANALYSIS) {
			missionStatus_ = MissionStatus::APPROVEDANALYSIS;
			this->setOrbitGoal();
			this->requestPathForNavigation();
		}
	}
	else if(commandType == CommandType::CANCELLED) {
		missionStatus_ = MissionStatus::CANCELLED;
		response->status = "cancelled";
		response->success = false;
		RCLCPP_INFO_STREAM(this->get_logger(), msg);
		this->publishIntermediaryStatus(msg, false);
		this->checkOverrideStatus();
	}
	else {
		std::string msg = "CommandType: None not valid";
		RCLCPP_ERROR_STREAM(this->get_logger(), msg.c_str());
		missionStatus_ = MissionStatus::UNKNOWN;
		publishIntermediaryStatus(msg, false);
	}
}

void MissionHandlerNode::checkOverrideStatus()
{
	if(missionStatus_ == MissionStatus::READY)
		return; // nothing to do as its not yet started

	if(missionStatus_ == MissionStatus::APPPROVEDINITIAL || missionStatus_ == MissionStatus::APPROVEDANALYSIS
		|| missionStatus_ == MissionStatus::CANCELLED || missionStatus_ == MissionStatus::UNKNOWN){
		this->cancelOverrideTimer();
		if(missionStatus_ == MissionStatus::CANCELLED){
			missionStatus_ = MissionStatus::READY;
			RCLCPP_DEBUG(this->get_logger(), "Mission status set: %d", static_cast<int>(missionStatus_));	
		}
		return;
	}
	RCLCPP_DEBUG(this->get_logger(), "In check override status");
	
	try {
		RCLCPP_DEBUG(this->get_logger(), "In check override status - %s", AgentUtils::toTaskTypeStr(taskType_).c_str());
		auto initialTaskType = hazardType_ == HazardType::FIBROUS_HAZARD ? TaskType::INITIAL_INSPECTION 
							: (hazardType_ == HazardType::DUST_PLUME ? TaskType::DUST_PLUME_ACTIONS : TaskType::NONE);

		this->cancelOverrideTimer();							
		if(initialTaskType == TaskType::NONE) {
			std::string msg = "No such task type";
			RCLCPP_ERROR(this->get_logger(), msg.c_str());
			missionStatus_ = MissionStatus::UNKNOWN;
			publishIntermediaryStatus(msg, false);
			return;
		}
		std::string msg;
		// if it had been approved / cancelled, it will not come here
		if((taskType_ == initialTaskType || taskType_ == TaskType::ASBESTOS_ANALYSIS)) 
		{
			msg = "No override received, Timeout reached. Proceding with mission!";
			RCLCPP_DEBUG_STREAM(this->get_logger(), msg);

			this->publishIntermediaryStatus(msg, true);
			missionStatus_ = MissionStatus::INPROGRESS;
			if(taskType_ == initialTaskType) {
				this->requestPathForNavigation();
			} else if(taskType_ == TaskType::ASBESTOS_ANALYSIS) {
				this->setOrbitGoal();
				this->requestPathForNavigation();
			}
		}	
	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occurred- ") + ex.what();
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		this->cancelOverrideTimer();
		publishIntermediaryStatus(msg, false);
		missionStatus_ = MissionStatus::UNKNOWN;
	}
}

void MissionHandlerNode::reachedTargetCallback(const transportMsgs::NavigationResult& resultMessage)
{
	RCLCPP_DEBUG(this->get_logger(), "In reach target callback: is return ?: %s", isReturn_ ? "True" : "False");
	if(!resultMessage.success)	{
		// if error in navigating, robot might be stuck in between so best to restart app
		std::string message = "Critical error during navigation. Did not reach target" + appRestartMsg_;
		RCLCPP_ERROR(this->get_logger(), message.c_str());
		publishIntermediaryStatus(message, false);
		missionStatus_ = MissionStatus::CRITICAL;
		return;
	}

	try {
		if(isReturn_) {
			this->setNextTaskType();
			if(tasksDone_) {
				isReturn_ = false; // set return to false
				RCLCPP_INFO_STREAM(this->get_logger(), resultMessage.message << " for " << hazardTypeStr_);
				tasksDone_ = false;
				this->sendParkingCommand();
			} else {
				this->waitForApprovalOrTimeout();
			}
		} else {
			// first task
			this->publishOnArrivalTask();
		}

	} catch(const std::runtime_error& ex) {
		auto message = std::string("Exception occurred- ") + ex.what();
		if(!isReturn_) {
			message += appRestartMsg_;
		}
		RCLCPP_ERROR(get_logger(), message.c_str());
		publishIntermediaryStatus(message, false);
		missionStatus_ = isReturn_ ? MissionStatus::UNKNOWN : MissionStatus::CRITICAL;
	}
}

void MissionHandlerNode::onArrivalTaskCompletion(
	transportMsgs::MissionStatus arrivalTaskCompletionMessage)
{
	RCLCPP_DEBUG(this->get_logger(), "In arrival task completion");
	isReturn_ = true;
	auto msg = AgentUtils::getMessage("Received message: ", arrivalTaskCompletionMessage.message, 
								 " for ", arrivalTaskCompletionMessage.hazard_type, " performing ",
								 	arrivalTaskCompletionMessage.task_type);
	if(!arrivalTaskCompletionMessage.success) {
		msg += appRestartMsg_;
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		missionStatus_ = MissionStatus::CRITICAL;		
		this->publishIntermediaryStatus(msg, false);
	
		return;
	}
	this->publishIntermediaryStatus(arrivalTaskCompletionMessage.message, arrivalTaskCompletionMessage.success,
									hazardType_ == HazardType::FIBROUS_HAZARD);
	RCLCPP_DEBUG(this->get_logger(), msg.c_str());
	this->startReturnToBase(arrivalTaskCompletionMessage);
}

void MissionHandlerNode::completedParking(transportMsgs::MissionCompleted parkingCompletedMessage)
{
	RCLCPP_DEBUG(this->get_logger(), "In completed Parking");
	std::string msg = parkingCompletedMessage.success ? "Parking completed" : "Parking failed";
	publishIntermediaryStatus(msg, parkingCompletedMessage.success);
	this->publishReturnToBaseStatus();
	missionStatus_ = MissionStatus::READY;
}

void MissionHandlerNode::publishIntermediaryStatus(
	std::string& message, const bool success, const bool requiresAnalysis)
{
	transportMsgs::MissionStatus missionStatusMessage;
	missionStatusMessage.hazard_type = AgentUtils::toHazardTypeString(hazardType_);
	missionStatusMessage.task_type = AgentUtils::toTaskTypeStr(taskType_);
	missionStatusMessage.message = message;
	missionStatusMessage.success = success;
	missionStatusMessage.requires_analysis = requiresAnalysis;

	intermediadyMissionStatusPub_->publish(missionStatusMessage);
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Intermediary status: " << message << " published.");
}

geometry_msgs::msg::Pose MissionHandlerNode::getReturnPose()
{
	auto qz = sin(agilexPose_.yaw / 2.0);
	auto qw = cos(agilexPose_.yaw / 2.0);

	auto pose = AgentUtils::getPose(agilexPose_, qz, qw);
	return pose;
}

void MissionHandlerNode::setOrbitGoal()
{
	auto robotPoseX = hazardPose_.position.x;
	auto robotPoseY = hazardPose_.position.y;
	auto hazardPoseX = hazardCenterPoint_.x;
	auto hazardPoseY = hazardCenterPoint_.y;

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Hazard Pose: " << hazardPoseX << " " << hazardPoseY);
	RCLCPP_DEBUG_STREAM(this->get_logger(), "Robot Pose: " << robotPoseX << " " << robotPoseY);
}

void MissionHandlerNode::requestPathForNavigation()
{
	try {
		RequestPathPlan::Goal request;
		RCLCPP_DEBUG_STREAM(this->get_logger(), "hazard pose: x: "  << hazardPose_.position.x << " y: " << hazardPose_.position.y );

		request.goal = isReturn_ ? returnPose_ : (taskType_ == TaskType::ASBESTOS_ANALYSIS ? hazardOrbitPose_ : hazardPose_);
		request.is_return = isReturn_;

		SendGoalOptions sendOptions;
		sendOptions.result_callback = std::bind(&MissionHandlerNode::handlePathPlanResult, this,
														std::placeholders::_1);

		pathPlannerPlugin_->planPath(request, sendOptions);													 
	} catch(const std::runtime_error& ex) {
		std::string msg = std::string("Exception occurred: ") + ex.what();
		if(isReturn_) {
			msg += appRestartMsg_;
		}
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		publishIntermediaryStatus(msg, false);
		missionStatus_ = isReturn_ ? MissionStatus::CRITICAL : MissionStatus::UNKNOWN;

	}
}

void MissionHandlerNode::sendNavigationCommand(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
	auto request = std::make_shared<Navigate::Request>();
	request->hazard_type = AgentUtils::toHazardTypeString(hazardType_);
	request->waypoints = waypoints;

	navigationPlugin_->handleNavigation(request);
}

void MissionHandlerNode::sendParkingCommand()
{
	RCLCPP_DEBUG(this->get_logger(), "In send parking command");
	try {
		parkingTfLookupCallTimer_ = this->create_wall_timer(std::chrono::milliseconds(10),
									[this](){
										RCLCPP_DEBUG(this->get_logger(), "In send parking command - looking up transform");
										auto tfOpt = AgentUtils::lookupTransform(tfBuffer_);
										
										if(tfOpt.has_value()) {
											RCLCPP_DEBUG(this->get_logger(), "In send parking command - lookupp transform worked");
											geometry_msgs::msg::TransformStamped currentTf = tfOpt.value();
											if(!parkingTfLookupCallTimer_->is_canceled())
												parkingTfLookupCallTimer_->cancel();
											auto request = std::make_shared<RotateToPosition::Request>();
											request->current_yaw = tf2::getYaw(currentTf.transform.rotation);
											request->target_yaw = AgentUtils::getParkingTargetHeading();

											navigationPlugin_->completeRotation(request);
										}
									});

		parkingTfLookupTimeoutTimer_ = this->create_wall_timer(std::chrono::minutes(navigationOverrideTimeoutInMinutes_),
										[this](){
											RCLCPP_DEBUG(this->get_logger(), "In send parking - timeout timer start");
											
											if(!parkingTfLookupCallTimer_->is_canceled()){
												RCLCPP_INFO(this->get_logger(), "TF lookup for parking timeout reached. So cannot park");
												parkingTfLookupCallTimer_->cancel();
												std::string msg {"Parking failed"};
												publishIntermediaryStatus(msg, false);
												this->publishReturnToBaseStatus();
												missionStatus_ = MissionStatus::UNKNOWN;
											}
											if(!parkingTfLookupTimeoutTimer_->is_canceled())
												parkingTfLookupTimeoutTimer_->cancel();
											});		
		RCLCPP_DEBUG(this->get_logger(), "Created parking timers");
	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occurred- ") + ex.what();
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		publishIntermediaryStatus(msg, false);
		missionStatus_ = MissionStatus::UNKNOWN;
	}								
}

void MissionHandlerNode::waitForApprovalOrTimeout()
{
	auto msg = AgentUtils::getMessage("Waiting: ", navigationOverrideTimeoutInMinutes_, 
										" minute(s) for supervisor override after return to base");
	RCLCPP_INFO_STREAM(this->get_logger(), msg);
	this->publishIntermediaryStatus(msg, true, true);							
	missionStatus_ = MissionStatus::PENDINGANALYSIS;
	overrideTimerStep2_ = this->create_wall_timer(std::chrono::minutes(navigationOverrideTimeoutInMinutes_),
													std::bind(&MissionHandlerNode::checkOverrideStatus, this));

}

void MissionHandlerNode::publishOnArrivalTask()
{
	auto taskTypeStr = AgentUtils::toTaskTypeStr(taskType_);
	transportMsgs::OnArrivalTask onArrivalTaskMsg {};
	onArrivalTaskMsg.hazard_type = hazardTypeStr_;
	onArrivalTaskMsg.task_type = taskTypeStr;
	onArrivalTaskMsg.estimated_hazard_diameter = estimatedHazardDiameter_;
	onArrivalTaskMsg.hazard_center_point = hazardCenterPoint_;
	
	onArrivalTaskStartPub_->publish(onArrivalTaskMsg);

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Published message on Arrival for: " 
										<< hazardTypeStr_ << " - " << taskTypeStr );
	auto msg = AgentUtils::getMessage("Arrived at the hazard and will start ", taskTypeStr);
	this->publishIntermediaryStatus(msg, true);
}

void MissionHandlerNode::setNextTaskType()
{
	switch(hazardType_) {
		case HazardType::DUST_PLUME:
			taskType_ = TaskType::NONE;
			tasksDone_ = true;
			isReturn_ = false;
			break;
		case HazardType::FIBROUS_HAZARD:
			if(taskType_ == TaskType::INITIAL_INSPECTION) {
				// for Phase 1
				taskType_ = TaskType::NONE;
				tasksDone_ = true;
				//taskType_ = TaskType::ASBESTOS_ANALYSIS;  // deliberately commented out for this phase
			} else if(taskType_ == TaskType::ASBESTOS_ANALYSIS) {
				taskType_ = TaskType::NONE;
				tasksDone_ = true;
			}
			break;
		default:
			taskType_ = TaskType::NONE;
			break;			
	}
}

void MissionHandlerNode::startReturnToBase(transportMsgs::MissionStatus missionStatusMessage)
{
	auto msg = AgentUtils::getMessage("Received message: ", missionStatusMessage.message, 
								 " for ", missionStatusMessage.hazard_type, " performing ",
								 	missionStatusMessage.task_type);
	if(!missionStatusMessage.success) {
		RCLCPP_ERROR_STREAM(this->get_logger(), msg);
		publishIntermediaryStatus(msg, false);
		return;
	}
	RCLCPP_INFO_STREAM(this->get_logger(), msg);
	try {
		returnPose_ = this->getReturnPose();
		isReturn_ = true;
		RCLCPP_INFO_STREAM(this->get_logger(), "On Arrival Task completed for " << hazardTypeStr_ << 
												": - - Navigating back to base...");
		this->requestPathForNavigation();
	} catch(const std::runtime_error& ex) {
		auto msg = std::string("Exception occurred- ") + ex.what();
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		publishIntermediaryStatus(msg, false);
	}
}

void MissionHandlerNode::publishReturnToBaseStatus()
{
	transportMsgs::BaseReturn baseReturnMessage {};
	baseReturnMessage.message = AgentUtils::getMessage("Returned to base after completion for mission for ", 
										hazardTypeStr_, ". Standing by for next mission.");
	baseReturnMessage.success = true;
	baseReturnPub_->publish(baseReturnMessage);
	
}

Edge MissionHandlerNode::computeEdge(float robotPosX, float robotPosY, float hazardCenterPosX, float hazardCenterPosY)
{
    auto dx = robotPosX - hazardCenterPosX; // robot offset in X (west = –ve)
	auto dy = robotPosY - hazardCenterPosY; // robot offset in Y (south = –ve)

	// Pick the axis with the *larger* absolute separation first.
    // That gives the shortest straight-line drive to the circle.
	if(abs(dx) > abs(dy)) 
		return dx < 0 ? Edge::WEST : Edge::EAST;
	if(abs(dy) > abs(dx)) 
		return dy < 0 ? Edge::SOUTH : Edge::NORTH;

	/*
		If they're equal (robot sits on a 45 ° line) choose the edge that
    	already puts the hazard on our *left* when heading straight in.
    	-  dx < 0  → robot is west  → drive east (edge = west)
    	-  dx ≥ 0  → robot is east  → drive west (edge = east)
	*/
	return dx <0 ? Edge::WEST : Edge::EAST;
}

void MissionHandlerNode::cancelOverrideTimer()
{
	if(taskType_ == TaskType::INITIAL_INSPECTION || 
		taskType_ == TaskType::DUST_PLUME_ACTIONS)  {
		if(overrideTimer_ != nullptr && !overrideTimer_->is_canceled())
			overrideTimer_->cancel();
	} else if(taskType_ == TaskType::ASBESTOS_ANALYSIS) {
		if(overrideTimerStep2_ != nullptr && !overrideTimerStep2_->is_canceled()) {
			overrideTimerStep2_->cancel();
		}
	}
}

void MissionHandlerNode::requestPathPlan()
{
	RequestPathPlan::Goal requestPathPlanGoal;

	rclcpp_action::Client<RequestPathPlan>::SendGoalOptions options;

	options.goal_response_callback = [this](auto goal_handle) {
		if (!goal_handle)
        {
			std::string msg = "Goal rejected";
            RCLCPP_ERROR(this->get_logger(), msg.c_str());
			publishIntermediaryStatus(msg, false);
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Goal accepted");
	};

	options.feedback_callback = [this](auto, 
										const std::shared_ptr<const RequestPathPlan::Feedback> feedback) {
		 RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());										
	};

	options.result_callback = std::bind(&MissionHandlerNode::handlePathPlanResult, 
								this, std::placeholders::_1);
	pathPlannerPlugin_->planPath(requestPathPlanGoal, options);
}

void MissionHandlerNode::handlePathPlanResult(const GoalHandleRequestPathPlan::WrappedResult &result)
{
	if(result.result->success)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Received waypoints: " << result.result->waypoints.size());
		this->sendNavigationCommand(result.result->waypoints);
	} else {
		auto message = result.result->message;
		RCLCPP_ERROR_STREAM(get_logger(), "Error whilst requesting for plan paths: " << message); 
		if(isReturn_) {
			message += appRestartMsg_;
		}
		publishIntermediaryStatus(message, false);
		missionStatus_ = isReturn_ ? MissionStatus::CRITICAL : MissionStatus::UNKNOWN;
	}

}
