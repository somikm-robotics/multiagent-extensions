#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

using Twist = geometry_msgs::msg::Twist;

class TwistRelayNode: public rclcpp::Node {
public:

    TwistRelayNode();

private:
    void relayCmd(const Twist& twistMsg);

    rclcpp::Publisher<Twist>::SharedPtr twistPublisher_;
    rclcpp::Subscription<Twist>::SharedPtr twistSubscription_;

};