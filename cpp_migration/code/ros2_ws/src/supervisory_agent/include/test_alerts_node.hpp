#include <rclcpp/rclcpp.hpp>
#include "transport_interfaces/msg/hazard_pose.hpp"
#include "shared_infrastructure/enums.hpp"

namespace transportMsgs = transport_interfaces::msg;

class TestAlertsNode : public rclcpp::Node 
{
public:
    TestAlertsNode();
    void cancelTimer();

private:
    HazardType hazardType_;  
    void publishHazard();

    rclcpp::Publisher<transportMsgs::HazardPose>::SharedPtr hazardPosePublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};