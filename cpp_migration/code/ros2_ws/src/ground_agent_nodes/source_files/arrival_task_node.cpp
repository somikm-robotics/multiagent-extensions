#include "arrival_task_node.hpp"

#include "agent_utils.hpp"

ArrivalTaskNode::ArrivalTaskNode() : Node ("arrival_task_node"),
                                     initialInspectionPluginLoader_("ground_agent_on_arrival_task_plugins", "InitialInspectionPluginBase"),
                                     orbitHazardPluginLoader_("ground_agent_on_arrival_task_plugins", "OrbitHazardManagerPluginBase")   
{
    // Publishers
    missionStatusPub_ = this->create_publisher<transportMsgs::MissionStatus>(
        "on_arrival_task_mission_status", 10);

    //Subscriptions
    onArrivalTaskStartSub_ = this->create_subscription<transportMsgs::OnArrivalTask>(
    "on_arrival_task", 10, [this](const transportMsgs::OnArrivalTask& result){
        this->onArrival(result);
    });

    onInspectionCompletedSub_ = this->create_subscription<transportMsgs::MissionCompleted>(
    "inspection_complete", 10, [this](const transportMsgs::MissionCompleted& result){
        this->onInitialInspectionComplete(result);
    });

    onOrbitCompletedSub_ = this->create_subscription<transportMsgs::MissionCompleted>(
    "orbit_complete", 10, [this](const transportMsgs::MissionCompleted& result){
        this->onOrbitComplete(result);
    });

    onDustPlumeActionsCompletedSub_ = this->create_subscription<transportMsgs::MissionStatus>(
    "dust_plume_actions_complete", 10, [this](const transportMsgs::MissionStatus& actionsCompletedMsg){
        this->onDustPlumeActionsComplete(actionsCompletedMsg);
    });

    // clients
  //  initialInspectionClient_ = this->create_client<InitialInspection>("initial_inspection");
  //  orbitHazardClient_ = this->create_client<OrbitHazard>("orbit_fibrous_hazard");

    // plugin
    initialInspectionPlugin_ = initialInspectionPluginLoader_.createSharedInstance("ground_agent_on_arrival_task_plugins/InitialInspectionPlugin");

    auto pluginType = AgentUtils::getCurrentOrbitHazardPlugin();
    if(pluginType == "nav2") {
        orbitHazardPlugin_ = orbitHazardPluginLoader_.createSharedInstance("ground_agent_on_arrival_task_plugins/Nav2OrbitHazardManagerPlugin");
    } else {
        orbitHazardPlugin_ = orbitHazardPluginLoader_.createSharedInstance("ground_agent_on_arrival_task_plugins/TwistCommanderOrbitHazardPlugin");
    }

    hazardType_ = HazardType::NONE;
    taskType_ = TaskType::NONE;
}

void ArrivalTaskNode::initialise()
{
    initialInspectionPlugin_->initialise(this->shared_from_this());
    orbitHazardPlugin_->initialise(this->shared_from_this());
}

void ArrivalTaskNode::onArrival(const transportMsgs::OnArrivalTask& arrivalTaskMsg)
{
    try {
        RCLCPP_INFO_STREAM(this->get_logger(), "On Arrival: Hazard Type: " << arrivalTaskMsg.hazard_type << " Task Type: " << arrivalTaskMsg.task_type);    
        //hazardType_ = AgentUtils::parseHazardType(arrivalTaskMsg.hazard_type);
        //taskType_ = AgentUtils::parseTaskType(arrivalTaskMsg.task_type);
        
        if(hazardType_ == HazardType::FIBROUS_HAZARD) {
            if(taskType_ == TaskType::INITIAL_INSPECTION) {
                auto initialInspectionRequest = std::make_shared<InitialInspection::Request>();
                initialInspectionRequest->hazard_type = arrivalTaskMsg.hazard_type;
                auto response = initialInspectionPlugin_->performInspection(initialInspectionRequest);
                RCLCPP_INFO_STREAM(this->get_logger(), "Inspection Inspection returned: " << response->message);    
            } else if (taskType_ == TaskType::ASBESTOS_ANALYSIS) {
                auto orbitHazardRequest = std::make_shared<OrbitHazard::Request>();
                orbitHazardRequest->hazard_type = arrivalTaskMsg.hazard_type;
                orbitHazardRequest->estimated_hazard_diameter = arrivalTaskMsg.estimated_hazard_diameter;
                orbitHazardRequest->hazard_center_point = arrivalTaskMsg.hazard_center_point;
                orbitHazardPlugin_->doHazardOrbit(orbitHazardRequest);
            }
        } else if (hazardType_ == HazardType::DUST_PLUME) {
            transportMsgs::StartMission startMissionMsg;
            startMissionMsg.message = "Starting Dust Plume Actions";
            startDustPlumeActionsMissionPub_->publish(startMissionMsg);
            RCLCPP_INFO_STREAM(this->get_logger(), startMissionMsg.message);
        }

    }catch(const std::runtime_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
    }
}

void ArrivalTaskNode::onDustPlumeActionsComplete(const transportMsgs::MissionStatus &dustPlumeActionsCompletedMsg)
{
    this->publishTaskCompleteMessage(dustPlumeActionsCompletedMsg.message, dustPlumeActionsCompletedMsg.success);
}

void ArrivalTaskNode::onOrbitComplete(const transportMsgs::MissionCompleted& orbitCompletedMsg)
{
    std::string msg = "Orbit of Fibrous hazard completed, ros2 bags collected for further analysis";
    this->publishTaskCompleteMessage(msg, orbitCompletedMsg.success);
}

void ArrivalTaskNode::onInitialInspectionComplete(const transportMsgs::MissionCompleted& inspectionCompletedMsg)
{
    std::string msg = "Inspection completed";
    this->publishTaskCompleteMessage(msg, inspectionCompletedMsg.success);
}

void ArrivalTaskNode::publishTaskCompleteMessage(const std::string& message, bool success)
{
    // auto taskType = AgentUtils::toTaskTypeStr(taskType_);
    // transportMsgs::MissionStatus missionStatusMessage;
    // missionStatusMessage.hazard_type = AgentUtils::toHazardTypeString(hazardType_);
    // missionStatusMessage.task_type = taskType;
    // missionStatusMessage.message = message;
    // missionStatusMessage.success = success;
    // missionStatusPub_->publish(missionStatusMessage);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Published Mission status message for " << taskType);
}

int main(int argc, char** argv) {

    auto node = std::make_shared<ArrivalTaskNode>();
    node->initialise();
    try {
        rclcpp::spin(node);
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}