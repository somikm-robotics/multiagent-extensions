#include "dust_plume_actions_node.hpp"

using namespace std::chrono_literals;

DustPlumeActionsNode::DustPlumeActionsNode() : Node("dust_plume_actions_node"),
                                               dustPlumeEstimationPluginLoader_("ground_agent_dust_plume_actions_plugins", "DustPlumeEstimationPluginBase"),
                                               dustSensorRelayPluginLoader_("ground_agent_dust_plume_actions_plugins", "DustSensorRelayPluginBase")
{
    missionStatusPub_ = this->create_publisher<transportMsgs::MissionStatus>("dust_plume_actions_complete", 10);    
    estimationCompletedSub_ = this->create_subscription<transportMsgs::DensityEstimationResult>("estimation_complete", 10,
                                [this](const transportMsgs::DensityEstimationResult& densityEstimationResultMsg){
                                    this->dustPlumeEstimationCompletedCallback(densityEstimationResultMsg);
                                });
    sensorRelayCompletedSub_ = this->create_subscription<transportMsgs::DustSensorResult>("dust_sensor_reading", 10,
                                    [this](const transportMsgs::DustSensorResult& dustSensorResultMsg){
                                        this->dustSensorRelayCompletedCallback(dustSensorResultMsg);
                                    });                                
    
    startMissionSub_ = this->create_subscription<transportMsgs::StartMission>( "start_dust_plume_actions_task", 10, 
        [this](const transportMsgs::StartMission& startMissionMsg){
        this->startTasks(startMissionMsg);
    });

    estimationCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensorRelayCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    started_ = false;
    estimationDone_ = false;
    sensorRelayDone_ = false;

    dustPlumeEstimationPlugin_ = dustPlumeEstimationPluginLoader_.createSharedInstance("DustPlumeEstimationPlugin");
    dustSensorRelayPlugin_ = dustSensorRelayPluginLoader_.createSharedInstance("DustSensorRelayPlugin");

    RCLCPP_INFO(this->get_logger(), "constructor done");
}

void DustPlumeActionsNode::initialise()
{
    RCLCPP_INFO(this->get_logger(), "In initialise");
    dustPlumeEstimationPlugin_->initialise(this->shared_from_this());
    dustSensorRelayPlugin_->initialise(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "completed initialise");
}

void DustPlumeActionsNode::startTasks(const transportMsgs::StartMission startMissionMsg)
{
    started_ = true;
    RCLCPP_INFO(this->get_logger(), "In start tasks");
    estimationTimer_ = this->create_wall_timer(100ms,[this]() {
        estimationTimer_->cancel();
        this->callDustPlumeEstimation();
    }, estimationCallbackGroup_);

    sensorRelayTimer_ = this->create_wall_timer(100ms, [this]() {
        sensorRelayTimer_->cancel();
        callDustSensorRelay();
    }, sensorRelayCallbackGroup_);
}

DustPlumeActionsNode::~DustPlumeActionsNode()
{
    RCLCPP_INFO(this->get_logger(), "in destructor");
    estimationCallbackGroup_.reset();
    sensorRelayCallbackGroup_.reset();
    dustSensorRelayPlugin_.reset();
    dustPlumeEstimationPlugin_.reset();

    RCLCPP_INFO(this->get_logger(), "destructor done");
}

void DustPlumeActionsNode::callDustPlumeEstimation()
{
    RCLCPP_INFO(this->get_logger(), "In call dust plume estimation");
    auto request = std::make_shared<PlumeDensity::Request>();
    dustPlumeEstimationPlugin_->estimatePlumeDensity(request);
}

void DustPlumeActionsNode::callDustSensorRelay()
{
    RCLCPP_INFO(this->get_logger(), "In call dust sensor result");
    dustSensorRelayPlugin_->publishDustSensorResult();
}

void DustPlumeActionsNode::notifyOnceDone(std::string message, bool success)
{
    message_ << " " << message << std::endl;
    missionSuccess_ = missionSuccess_.has_value() ? missionSuccess_.value() && success : success;

    if(estimationDone_ && sensorRelayDone_) {
        RCLCPP_INFO(this->get_logger(), "In notify: %s", message_.str().c_str());
        transportMsgs::MissionStatus missionStatusMsg;
        missionStatusMsg.success = missionSuccess_.value();
        missionStatusMsg.message = message_.str();
        missionStatusPub_->publish(missionStatusMsg);
    }
}

void DustPlumeActionsNode::dustPlumeEstimationCompletedCallback(const transportMsgs::DensityEstimationResult& densityEstimationResultMsg)
{
    auto message = densityEstimationResultMsg.success ? "Dust Plume Estimation Completed Successfully"
                                                                       :  "Dust Plume Estimation was unsuccessful" ;
    RCLCPP_INFO(this->get_logger(), message);
    estimationDone_ = true;
    notifyOnceDone(message, densityEstimationResultMsg.success);

}

void DustPlumeActionsNode::dustSensorRelayCompletedCallback(const transportMsgs::DustSensorResult& dustSensorResultMsg)
{
    
    auto message = dustSensorResultMsg.success ? "Dust Sensor Relay Completed Successfully"
                                                                       :  "Dust Sensor Relay was unsuccessful" ;
    RCLCPP_INFO(this->get_logger(), message);
    sensorRelayDone_ = true;
    notifyOnceDone(message, dustSensorResultMsg.success);
}


