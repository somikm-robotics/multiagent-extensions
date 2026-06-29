#include "dust_plume_actions_node.hpp"

DustPlumeActionsNode::DustPlumeActionsNode() : Node("dust_plume_actions_node"),
                                               dustPlumeEstimationPluginLoader_("ground_agent_dust_plume_actions_plugins", "DustPlumeEstimationPluginBase"),
                                               dustSensorRelayPluginLoader_("ground_agent_dust_plume_actions_plugins", "DustSensorRelayPluginBase")
{
    missionStatusPub_ = this->create_publisher<transportMsgs::MissionStatus>("dust_plume_actions_complete", 10);    
    startMissionSub_ = this->create_subscription<transportMsgs::StartMission>( "start_dust_plume_actions_task", 10, 
        [this](const transportMsgs::StartMission& startMissionMsg){
        this->startTasks(startMissionMsg);
    });

    estimationCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sensorRelayCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    started_ = false;
    estimationDone_ = false;
    sensorRelayDone_ = false;

    dustPlumeEstimationPlugin_ = dustPlumeEstimationPluginLoader_.createSharedInstance("ground_agent_dust_plume_actions_plugins/DustPlumeEstimationPlugin");
    dustSensorRelayPlugin_ = dustSensorRelayPluginLoader_.createSharedInstance("ground_agent_dust_plume_actions_plugins/DustSensorRelayPlugin");
}

void DustPlumeActionsNode::initialise()
{
    dustPlumeEstimationPlugin_->initialise(this->shared_from_this());
    dustSensorRelayPlugin_->initialise(this->shared_from_this());
}

void DustPlumeActionsNode::startTasks(const transportMsgs::StartMission startMissionMsg)
{
    started_ = true;
    callDustPlumeEstimation();
    callDustSensorRelay();
}

void DustPlumeActionsNode::callDustPlumeEstimation()
{
    auto request = std::make_shared<PlumeDensity::Request>();
    dustPlumeEstimationPlugin_->estimatePlumeDensity(request);
   
}

void DustPlumeActionsNode::callDustSensorRelay()
{
    dustSensorRelayPlugin_->publishDustSensorResult();
}



void DustPlumeActionsNode::notifyOnceDone(std::string message, bool success)
{
    message_ << message << std::endl;
    missionSuccess_ = missionSuccess_.has_value() ? missionSuccess_.value() && success : success;

    if(estimationDone_ && sensorRelayDone_) {
        transportMsgs::MissionStatus missionStatusMsg;
        missionStatusMsg.success = missionSuccess_.value();
        missionStatusMsg.message = message_.str();
        missionStatusPub_->publish(missionStatusMsg);
    }
   
}

void DustPlumeActionsNode::dustPlumeEstimationCompletedCallback(transportMsgs::MissionCompleted dustPlumeEstimationCompletedMsg)
{
    auto message = dustPlumeEstimationCompletedMsg.success ? "Dust Plume Estimation Completed Successfully"
                                                                       :  "Dust Plume Estimation was unsuccessful" ;
    notifyOnceDone(message, dustPlumeEstimationCompletedMsg.success);

}

void DustPlumeActionsNode::dustSensorRelayCompletedCallback(transportMsgs::MissionCompleted dustSensorRelayCompletedMsg)
{
    auto message = dustSensorRelayCompletedMsg.success ? "Dust Sensor Relay Completed Successfully"
                                                                       :  "Dust Sensor Relay was unsuccessful" ;
    notifyOnceDone(message, dustSensorRelayCompletedMsg.success);
}


int main(int argc, char** argv) {

    auto node = std::make_shared<DustPlumeActionsNode>();
    node->initialise();
    try {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}