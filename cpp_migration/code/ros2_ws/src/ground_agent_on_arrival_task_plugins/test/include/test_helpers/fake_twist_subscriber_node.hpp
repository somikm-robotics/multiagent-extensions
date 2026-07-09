#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"


class FakeTwistSubscriberNode : public rclcpp::Node {
public:
   
    FakeTwistSubscriberNode();

private:

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twistMsg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub_;
    inline static const std::string kCmdVelTopic = "/cmd_vel";
};