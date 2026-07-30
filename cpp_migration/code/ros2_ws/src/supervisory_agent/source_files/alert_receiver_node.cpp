#include "alert_receiver_node.hpp"

AlertReceiverNode::AlertReceiverNode() : Node("alert_receiver_node")
{
    alertsPub_ = this->create_publisher<transportMsgs::UIAlerts>("/supervisor/alert", 10);
  
    hazardDetectedSub_ = this->create_subscription<transportMsgs::HazardPose>(
        "hazard_detected", 10, [this](const transportMsgs::HazardPose& hazardDetectedMsg) {
            this->onHazardDetected(hazardDetectedMsg);
        });

    // missionCompletedSub_ = this->create_subscription<transportMsgs::MissionCompleted>(
    //                         "inspection_complete", 10, [this](const transportMsgs::MissionCompleted& completedMsg){
    //                             this->onMissionCompleted(completedMsg);
    //                         });        

    missionStatusSub_ = this->create_subscription<transportMsgs::MissionStatus>(
                            "intermediary_mission_status", 10, [this](
                                const transportMsgs::MissionStatus& statusMsg) {
                                    this->onMissionStatus(statusMsg);
    });
    
    returnToBaseSub_ = this->create_subscription<transportMsgs::BaseReturn>(
                        "return_to_base_status", 10, [this](const transportMsgs::BaseReturn& returnMsg){
                            this->onReturnToBase(returnMsg);
                        });

    densityEstimationResultSub_ = this->create_subscription<transportMsgs::DensityEstimationResult>(
                                    "estimation_complete", 10, [this](const transportMsgs::DensityEstimationResult& resultMsg){
                                        this->onDensityEstimationResult(resultMsg);
                                    });                       

    dustSensorResultSub_ = this->create_subscription<transportMsgs::DustSensorResult>(
                            "dust_sensor_reading", 10, [this](
                                const transportMsgs::DustSensorResult& resultMsg){
                                    this->onDustSensorResult(resultMsg);
                            });

}

void AlertReceiverNode::onHazardDetected(const transportMsgs::HazardPose &hazardDetectedMsg)
{
    auto msg = "Hazard Detected: " + hazardDetectedMsg.hazard_type;;
    publishStatus(msg, true, false, true);
}

void AlertReceiverNode::onMissionStatus(const transportMsgs::MissionStatus &missionStatusMsg)
{
    publishStatus(missionStatusMsg.message, false, missionStatusMsg.requires_analysis, false);
}

void AlertReceiverNode::onReturnToBase(const transportMsgs::BaseReturn &returnToBaseMsg)
{
    publishStatus(returnToBaseMsg.message, false, false, true, true);
}

void AlertReceiverNode::onDensityEstimationResult(const transportMsgs::DensityEstimationResult &resultMsg)
{
    publishStatus(resultMsg.message, false, false);
}

void AlertReceiverNode::onDustSensorResult(const transportMsgs::DustSensorResult &resultMsg)
{
    publishStatus(resultMsg.message, false, false);
}

void AlertReceiverNode::publishStatus(std::string message, bool enableInspection, bool enableAnalysis, 
                                        bool isStatus, bool enableScan)
{
    transportMsgs::UIAlerts alertsMsg;
    alertsMsg.message = message;
    alertsMsg.is_status = isStatus;
    alertsMsg.enable_inspection = enableInspection;
    alertsMsg.enable_analysis = enableAnalysis;
    alertsMsg.enable_scan = enableScan;

    alertsPub_->publish(alertsMsg);
    RCLCPP_INFO(this->get_logger(), "Published: %s", alertsMsg.message.c_str());
}
