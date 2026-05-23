#include "mission_handler_node.h"
#include <stdexcept>
#include <functional>

MissionHandlerNode::MissionHandlerNode()  : Node ("mission_handler_node")
{
	//Publishers
	_onArrivalTaskStartPub = this->create_publisher<transportMsgs::OnArrivalTask>(
        "on_arrival_task", 10);
	_baseReturnPub = this->create_publisher<transportMsgs::BaseReturn>(
        "return_to_base_status", 10);
	_intermediadyMissionStatusPub = this->create_publisher<transportMsgs::MissionStatus>(
        "intermediary_mission_status", 10);

	//Subscribers
	_hazardDetectedSub = this->create_subscription<transportMsgs::HazardPose>(
    "hazard_detected", 10, [this](const transportMsgs::HazardPose& result){
        this->hazardDetectedCallback(result);
    });
	_navigationResultSub = this->create_subscription<transportMsgs::NavigationResult>(
    "reached_target", 10, [this](const transportMsgs::NavigationResult& result){
        this->reachedTargetCallback(result);
    });
	_onArrivalTaskCompletionSub = this->create_subscription<transportMsgs::OnArrivalTask>(
    "on_arrival_mission_status", 10, [this](const transportMsgs::OnArrivalTask& result){
        this->onArrivalTaskCompletion(result);
    });

	//Services
	_overrideMissionService = this->create_service<OverrideMission>(
            "override_mission", 
                [this] (std::shared_ptr<OverrideMission::Request> request, 
                    std::shared_ptr<OverrideMission::Response> response) {
                    this->handleOverride(request, response
                    );        
                } );

	_pathPlanResultService = this->create_service<PathPlanResult>(
            "receive_path_result", 
                [this] (std::shared_ptr<PathPlanResult::Request> request, 
                    std::shared_ptr<PathPlanResult::Response> response) {
                    this->handlePathResult(request, response
                    );        
                } );
	
	// Clients
	_requestPathPlanClient = this->create_client<RequestPathPlan>("request_path_plan");
	_navigateRequestClient = this->create_client<NavigateRequest>("navigate_to_pose");
}

void MissionHandlerNode::handleOverride(std::shared_ptr<OverrideMission::Request> request, std::shared_ptr<OverrideMission::Response> response)
{
}

void MissionHandlerNode::handlePathResult(std::shared_ptr<PathPlanResult::Request> request, std::shared_ptr<PathPlanResult::Response> response)
{
}



void MissionHandlerNode::hazardDetectedCallback(const transportMsgs::HazardPose& hazardPose)
{
}

void MissionHandlerNode::reachedTargetCallback(const transportMsgs::NavigationResult& result)
{
}

void MissionHandlerNode::onArrivalTaskCompletion(
	transportMsgs::OnArrivalTask arrivalTaskMessage)
{
	
}

void MissionHandlerNode::handleNavigationResponse(
	rclcpp::Client<NavigateRequest>::SharedFuture future)
{
	auto response = future.get();
	// TODO: implementation
}

void MissionHandlerNode::handlePathPlanResponse(
	rclcpp::Client<RequestPathPlan>::SharedFuture future)
{
	auto response = future.get();
	// TODO: implementation
}

void MissionHandlerNode::checkOverrideStatus()
{
}

void MissionHandlerNode::publishIntermediaryStatus(
	std::string& message, const bool toggleCommand)
{
	//TODO: Implementation - Only flow done
	transportMsgs::MissionStatus missionStatusMessage;
	_intermediadyMissionStatusPub->publish(missionStatusMessage);
}

geometry_msgs::msg::Pose& MissionHandlerNode::getReturnPose()
{
	throw std::logic_error("Not yet implemeted");
}

void MissionHandlerNode::setOrbitGoal()
{
}

void MissionHandlerNode::requestPathForNavigation()
{
	// TO Do implementation. Just adding flow for now
	auto request = std::make_shared<RequestPathPlan::Request>();
	_requestPathPlanClient->async_send_request(
		request, [this](rclcpp::Client<RequestPathPlan>::SharedFuture future) {
			this->handlePathPlanResponse(future);
		}
	);
}

void MissionHandlerNode::sendNavigationCommand(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
	auto request = std::make_shared<NavigateRequest::Request>();
	_navigateRequestClient->async_send_request(
		request, [this](rclcpp::Client<NavigateRequest>::SharedFuture future) {
			this->handleNavigationResponse(future);
		}
	);
}

void MissionHandlerNode::waitForApprovalOrTimeout()
{
}

void MissionHandlerNode::publishOnArrivalTask()
{
	
	//TODO: Implementation - Only flow done
	transportMsgs::OnArrivalTask onArrivalTaskMsg {};
	
	_onArrivalTaskStartPub->publish(onArrivalTaskMsg);
}

void MissionHandlerNode::setNextTaskType()
{
}

void MissionHandlerNode::startReturnToBase(transportMsgs::OnArrivalTask arrivalTaskMessage)
{
}

void MissionHandlerNode::publishReturnToBaseStatus()
{
	//TODO: Implementation - Only flow done
	transportMsgs::BaseReturn baseReturnMessage {};
	_baseReturnPub->publish(baseReturnMessage);
}

int main(int argc, char** argv) {

    auto node = std::make_shared<MissionHandlerNode>();

    try {
        rclcpp::spin(node);
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}
