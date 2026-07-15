#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

class FakeTFPublisherNode : public rclcpp::Node {
public:
    FakeTFPublisherNode();

private:
    void broadcast();

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    float x_{10.0};
    inline static float incrementCtr{-0.5f};
    std::vector<float> xPositions_ { 5.0, 6.0, 7.0, 8.0, 9.0, 9.7};
    int currentPosCtr_{0};
 };