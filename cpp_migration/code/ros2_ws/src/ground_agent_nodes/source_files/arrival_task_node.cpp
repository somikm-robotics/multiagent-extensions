#include "arrival_task_node.h"

ArrivalTaskNode::ArrivalTaskNode() : Node ("arrival_task_node")
{
    // Publishers
    _missionStatusPub = this->create_publisher<transportMsgs::MissionStatus>(
        "on_arrival_task_mission_status", 10);

    //Subscriptions
    _onArrivalTaskStartSub = this->create_subscription<transportMsgs::OnArrivalTask>(
    "on_arrival_task", 10, [this](const transportMsgs::OnArrivalTask& result){
        this->onArrival(result);
    });

    _onInspectionCompletedSub = this->create_subscription<transportMsgs::MissionCompleted>(
    "inspection_complete", 10, [this](const transportMsgs::MissionCompleted& result){
        this->onInitialInspectionComplete(result);
    });

    _onOrbitCompletedSub = this->create_subscription<transportMsgs::MissionCompleted>(
    "orbit_complete", 10, [this](const transportMsgs::MissionCompleted& result){
        this->onOrbitComplete(result);
    });

    _onDensityEstimationCompletedSub = this->create_subscription<transportMsgs::DensityEstimationResult>(
    "estimation_complete", 10, [this](const transportMsgs::DensityEstimationResult& result){
        this->onDensityEstimationComplete(result);
    });

    // clients
    _initialInspectionClient = this->create_client<InitialInspection>("initial_inspection");
    _orbitHazardClient = this->create_client<OrbitHazard>("orbit_fibrous_hazard");
}

void ArrivalTaskNode::onArrival(transportMsgs::OnArrivalTask& arrivalTask)
{
    // TO Do implementation. Just adding flow for now
	auto initialInspectionRequest = std::make_shared<InitialInspection::Request>();
	_requestPathPlanClient->async_send_request(initialInspectionRequest);

    auto orbitHazardRequest = std::make_shared<OrbitHazard::Request>();
	_orbitHazardClient->async_send_request(orbitHazardRequest);

    auto plumeDensityRequest = std::make_shared<PlumeDensity::Request>();
	_plumeDensityClient->async_send_request(plumeDensityRequest);
}

void ArrivalTaskNode::onDensityEstimationComplete(transportMsgs::DensityEstimationResult& result)
{
}

void ArrivalTaskNode::onOrbitComplete(transportMsgs::MissionCompleted& orbitCompleted)
{
}

void ArrivalTaskNode::onInitialInspectionComplete(transportMsgs::MissionCompleted& inspectionCompleted)
{
}

void ArrivalTaskNode::publishTaskCompleteMessage(std::string& message, bool success)
{
    //TODO: Implementation - Only flow done
    transportMsgs::MissionStatus missionStatusMessage;
    _missionStatusPub->publish(missionStatusMessage);
}

int main(int argc, char** argv) {

    auto node = std::make_shared<ArrivalTaskNode>();

    try {
        rclcpp::spin(node);
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}