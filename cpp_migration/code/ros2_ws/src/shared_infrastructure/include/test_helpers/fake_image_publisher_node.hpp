#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class FakeImagePublisherNode : public rclcpp::Node {
public:
    FakeImagePublisherNode(); 

private:

    void publishImage();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
    rclcpp::TimerBase::SharedPtr pubTimer_;
    inline static const std::string CAMERA_TOPIC =
        R"(/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image)";
};