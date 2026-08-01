#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <memory>

using NavSatFix = sensor_msgs::msg::NavSatFix;

class GPSCovarianceInjectorNode : public rclcpp::Node {
public:
    GPSCovarianceInjectorNode();
    

private:
    void inject(NavSatFix& navSatMsg);

    rclcpp::Publisher<NavSatFix>::SharedPtr navSatMsgPublisher_;
    rclcpp::Subscription<NavSatFix>::SharedPtr navSatMsgSubscriber_;
};