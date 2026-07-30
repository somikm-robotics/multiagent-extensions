#include "test_alerts_node.hpp"
#include<chrono>

#include "agent_utils.hpp"

TestAlertsNode::TestAlertsNode() : Node("test_alerts_node")
{
    hazardPosePublisher_ = this->create_publisher<transportMsgs::HazardPose>(
                            "hazard_detected", 10);
    hazardType_ = HazardType::DUST_PLUME;                            

    timer_ = this->create_wall_timer(std::chrono::minutes(1), 
                            std::bind(&TestAlertsNode::publishHazard, this));                            
    RCLCPP_INFO(this->get_logger(), "created timer");
}

void TestAlertsNode::cancelTimer()
{
    if(!timer_->is_canceled())
        timer_->cancel();
}

void TestAlertsNode::publishHazard()
{
    transportMsgs::HazardPose hazardPose;
    hazardType_ = hazardType_ == HazardType::DUST_PLUME 
                            ? HazardType::FIBROUS_HAZARD : HazardType::DUST_PLUME;
    hazardPose.hazard_type =  AgentUtils::toHazardTypeString(hazardType_);
    
   // hazardPosePublisher_->publish(hazardPose);
   // RCLCPP_INFO(this->get_logger(), "published: %s", hazardPose.hazard_type.c_str());
}


int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestAlertsNode>();
    try 
    {
        rclcpp::spin(node);
    } catch(const std::exception &ex) 
    {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    
    node->cancelTimer();

    rclcpp::shutdown();

    return 0;
}