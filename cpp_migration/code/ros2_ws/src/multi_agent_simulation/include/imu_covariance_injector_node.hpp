#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

using Imu = sensor_msgs::msg::Imu;

class ImuCovarianceInjectorNode : public rclcpp::Node {
public:
    ImuCovarianceInjectorNode();
private:
    void imuCallback(Imu& imuMsg);

    rclcpp::Publisher<Imu>::SharedPtr imuPublisher_;
    rclcpp::Subscription<Imu>::SharedPtr imuSubcription_;
};