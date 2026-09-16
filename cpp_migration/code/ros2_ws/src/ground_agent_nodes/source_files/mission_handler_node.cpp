#include "mission_handler_node.hpp"
#include <stdexcept>
#include <functional>
#include <iostream>
#include <chrono>
#include <cmath>
#include "agent_utils.hpp"

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
	approved_ = std::nullopt;
	orbitApproved_ = std::nullopt;							
	hazardReceived_ = false;
	hazardType_ = HazardType::NONE;
	taskType_ = TaskType::NONE;
	estimatedHazardDiameter_ = 0.0;
	serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
	navigationOverrideTimeoutInMinutes_ = AgentUtils::getNavigationOverrideTimeoutFromConfig();
	tasksDone_ = false;
	isAtBase_ = true;
	isReturn_ = false;
}

void MissionHandlerNode::initialise()
{
	navigationPlugin_->initialise(this->shared_from_this());
	pathPlannerPlugin_->initialise(this->shared_from_this());
}

void MissionHandlerNode::hazardDetectedCallback(const transportMsgs::HazardPose& hazardPoseMessage)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "Ground Agent received " << hazardPoseMessage.hazard_type);


	if(!isAtBase_) {
		RCLCPP_INFO(this->get_logger(), "Hazard Processing in progress");
		return;
	}

	RCLCPP_INFO_STREAM(this->get_logger(), "hazard center point: x: " << hazardPoseMessage.hazard_center_point.x << " y: " << hazardPoseMessage.hazard_center_point.y );
	
	RCLCPP_INFO_STREAM(this->get_logger(), "hazard pose: x: " << hazardPoseMessage.pose.position.x << " y: " << hazardPoseMessage.pose.position.y );

	hazardReceived_ = true;
	hazardTypeStr_ = hazardPoseMessage.hazard_type;
	hazardType_ = AgentUtils::parseHazardType(hazardPoseMessage.hazard_type);
	if(hazardType_ == HazardType::NONE) {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid Hazard type- " << hazardTypeStr_);
	}

	taskType_ = hazardType_ == HazardType::FIBROUS_HAZARD ? TaskType::INITIAL_INSPECTION 
														: TaskType::DUST_PLUME_ACTIONS;
								
	hazardPose_ = hazardPoseMessage.pose;
	estimatedHazardDiameter_ = hazardPoseMessage.estimated_hazard_diameter;
	hazardCenterPoint_ = hazardPoseMessage.hazard_center_point;
	agilexPose_ = AgentUtils::getAgilexBasePose();

	try
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting " << navigationOverrideTimeoutInMinutes_ << " minute(s)");
		overrideTimer_ = this->create_wall_timer(std::chrono::minutes(navigationOverrideTimeoutInMinutes_),
													std::bind(&MissionHandlerNode::checkOverrideStatus, this));
		RCLCPP_INFO(this->get_logger(), "Created wall timer");
	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
	}
}

void MissionHandlerNode::handleOverride(std::shared_ptr<OverrideMission::Request> request, std::shared_ptr<OverrideMission::Response> response)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "Ground Agent received " << request->command);

	auto commandType = AgentUtils::parseCommandType(request->command);
	std::string msg = commandType == CommandType::APPROVED ? "Approved! Dispatching mission."
													: "Cancelled! Supervisor aborted the mission";
	if(commandType == CommandType::APPROVED) {
		response->status = "Dispatched";
		response->success = true;
		this->publishIntermediaryStatus(msg, false);
		RCLCPP_INFO_STREAM(this->get_logger(), msg);

		if(hazardReceived_) {
			if(taskType_ == TaskType::INITIAL_INSPECTION
				|| taskType_ == TaskType::DUST_PLUME_ACTIONS) {
				approved_ = true;
				this->requestPathForNavigation();
			}
			else if(taskType_ == TaskType::ASBESTOS_ANALYSIS) {
				orbitApproved_ = true;
				this->setOrbitGoal();
				this->requestPathForNavigation();
			}
			hazardReceived_ = false;
		}
	} else if(commandType == CommandType::CANCELLED) {
		approved_ = false;
		if(taskType_ == TaskType::ASBESTOS_ANALYSIS)
			orbitApproved_ = false;
		response->status = "cancelled";
		response->success = false;
		RCLCPP_INFO_STREAM(this->get_logger(), msg);
		hazardReceived_ = false;
		this->publishIntermediaryStatus(msg, false);
	}

}

void MissionHandlerNode::checkOverrideStatus()
{
	//RCLCPP_INFO(this->get_logger(), "In check override status");
	if(!hazardReceived_)
		return; // nothing to do
	try {
		RCLCPP_INFO(this->get_logger(), "In check override status - %s orbit Approved- %s", AgentUtils::toTaskTypeStr(taskType_).c_str(), orbitApproved_.has_value() ? "True": "False");
		auto initialTaskType = hazardType_ == HazardType::FIBROUS_HAZARD ? TaskType::INITIAL_INSPECTION 
							: (hazardType_ == HazardType::DUST_PLUME ? TaskType::DUST_PLUME_ACTIONS : TaskType::NONE);

		std::string msg;
		if((taskType_ == initialTaskType && !approved_.has_value())
			|| (taskType_ == TaskType::ASBESTOS_ANALYSIS && !orbitApproved_.has_value())) 
		{
			msg = "No override received, Timeout reached. Proceding with mission!";
			RCLCPP_INFO_STREAM(this->get_logger(), msg);

			if(taskType_ == initialTaskType)  {
				if(!overrideTimer_->is_canceled())
					overrideTimer_->cancel();
			} else {
				if(overrideTimerStep2_ != nullptr && !overrideTimerStep2_->is_canceled()) {
					overrideTimerStep2_->cancel();
				}
			}
			this->publishIntermediaryStatus(msg, false);

			if(taskType_ == TaskType::INITIAL_INSPECTION) {
				this->requestPathForNavigation();
			} else {
				this->setOrbitGoal();
				this->requestPathForNavigation();
			}
		}	
	} catch(const std::runtime_error& ex) {
			RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
	}	
	hazardReceived_ = false;
	approved_ = std::nullopt;
	orbitApproved_ = std::nullopt;							
	
}

void MissionHandlerNode::reachedTargetCallback(const transportMsgs::NavigationResult& resultMessage)
{
	RCLCPP_INFO(this->get_logger(), "In reach target callback: is return ?: %s", isReturn_ ? "True" : "False");
	if(!resultMessage.success)	{
		RCLCPP_ERROR(this->get_logger(), "Some error in navigation. Did not reach target");
		return;
	}

	try {
		if(isReturn_) {
			this->setNextTaskType();
			if(tasksDone_) {
				RCLCPP_INFO_STREAM(this->get_logger(), resultMessage.message << " for " << hazardTypeStr_);
				this->publishReturnToBaseStatus();
			} else {
				isReturn_ = false;
				this->waitForApprovalOrTimeout();
			}
		} else {
			// first task
			this->publishOnArrivalTask();
		}

	} catch(const std::runtime_error& ex) {

	}
}

void MissionHandlerNode::onArrivalTaskCompletion(
	transportMsgs::MissionStatus arrivalTaskCompletionMessage)
{
	//RCLCPP_INFO(this->get_logger(), "In arrival task completion");
	isReturn_ = true;
	auto msg = AgentUtils::getMessage("Received message: ", arrivalTaskCompletionMessage.message, 
								 " for ", arrivalTaskCompletionMessage.hazard_type, " performing ",
								 	arrivalTaskCompletionMessage.task_type);
	if(!arrivalTaskCompletionMessage.success) {
		RCLCPP_ERROR(this->get_logger(), msg.c_str());
		return;
	}
	RCLCPP_INFO(this->get_logger(), msg.c_str());
	this->publishIntermediaryStatus(arrivalTaskCompletionMessage.message, 
				hazardType_ == HazardType::FIBROUS_HAZARD);

	this->startReturnToBase(arrivalTaskCompletionMessage);
}


void MissionHandlerNode::publishIntermediaryStatus(
	std::string& message, const bool requiresAnalysis)
{
	transportMsgs::MissionStatus missionStatusMessage;
	missionStatusMessage.hazard_type = AgentUtils::toHazardTypeString(hazardType_);
	missionStatusMessage.task_type = AgentUtils::toTaskTypeStr(taskType_);
	missionStatusMessage.message = message;
	missionStatusMessage.success = true;
	missionStatusMessage.requires_analysis = requiresAnalysis;

	intermediadyMissionStatusPub_->publish(missionStatusMessage);
	RCLCPP_INFO_STREAM(this->get_logger(), "Intermediary status: " << message << " published.");
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

	RCLCPP_INFO_STREAM(this->get_logger(), "Hazard Pose: " << hazardPoseX << " " << hazardPoseY);
	RCLCPP_INFO_STREAM(this->get_logger(), "Robot Pose: " << robotPoseX << " " << robotPoseY);

}

void MissionHandlerNode::requestPathForNavigation()
{

	try {
		RequestPathPlan::Goal request;
		RCLCPP_INFO_STREAM(this->get_logger(), "hazard pose: x: "  << hazardPose_.position.x << " y: " << hazardPose_.position.y );

		request.goal = isReturn_ ? returnPose_ : (taskType_ == TaskType::ASBESTOS_ANALYSIS ? hazardOrbitPose_ : hazardPose_);
		//request.is_return = isReturn_;

		SendGoalOptions sendOptions;
		sendOptions.result_callback = std::bind(&MissionHandlerNode::handlePathPlanResult, this,
														std::placeholders::_1);
		sendOptions.goal_response_callback = [this] (GoalHandle::SharedPtr goalHandle) {
			if(!goalHandle) {
				RCLCPP_ERROR(this->get_logger(), "Path Plan failed");
			}
			RCLCPP_INFO(this->get_logger(), "Path Plan failed");
		};
		sendOptions.feedback_callback = [this] (GoalHandle::SharedPtr goalHaldle, 
										const std::shared_ptr<const RequestPathPlan::Feedback> feedback) {
			RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
		};

		pathPlannerPlugin_->plan_path(request, sendOptions);													 
	} catch(const std::runtime_error& ex) {
		RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
	}
}

void MissionHandlerNode::sendNavigationCommand(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
	auto request = std::make_shared<Navigate::Request>();
	request->hazard_type = AgentUtils::toHazardTypeString(hazardType_);
	request->waypoints = waypoints;

	navigationPlugin_->handleNavigation(request);
}

void MissionHandlerNode::waitForApprovalOrTimeout()
{
	auto msg = AgentUtils::getMessage("Waiting: ", navigationOverrideTimeoutInMinutes_, 
										" minute(s) for supervisor override after return to base");
	RCLCPP_INFO_STREAM(this->get_logger(), msg);
	this->publishIntermediaryStatus(msg, true);							
	hazardReceived_ = true;
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

	RCLCPP_INFO_STREAM(this->get_logger(), "Published message on Arrival for: " 
										<< hazardTypeStr_ << " - " << taskTypeStr );
	auto msg = AgentUtils::getMessage("Arrived at the hazard and will start ", taskTypeStr);
	this->publishIntermediaryStatus(msg, false);
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
				taskType_ = TaskType::ASBESTOS_ANALYSIS;
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
		RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
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

void MissionHandlerNode::requestPathPlan()
{
	RequestPathPlan::Goal requestPathPlanGoal;

	rclcpp_action::Client<RequestPathPlan>::SendGoalOptions options;

	options.goal_response_callback = [this](auto goal_handle) {
		if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted");
	};

	options.feedback_callback = [this](auto, 
										const std::shared_ptr<const RequestPathPlan::Feedback> feedback) {
		 RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());										
	};

	options.result_callback = std::bind(&MissionHandlerNode::handlePathPlanResult, 
								this, std::placeholders::_1);
	pathPlannerPlugin_->plan_path(requestPathPlanGoal, options);
}

void MissionHandlerNode::handlePathPlanResult(const GoalHandleRequestPathPlan::WrappedResult &result)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "Received waypoints: " << result.result->waypoints.size());
	this->sendNavigationCommand(result.result->waypoints);
}
