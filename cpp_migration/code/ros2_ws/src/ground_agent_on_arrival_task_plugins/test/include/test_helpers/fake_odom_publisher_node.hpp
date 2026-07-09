#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"

class FakeOdomPublisherNode: public rclcpp::Node {
public:
    FakeOdomPublisherNode();
    
private:

    void publishOdomMessage();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::TimerBase::SharedPtr pubTimer_;
    float yaw_{0.0f};
    inline static float incrementCtr{0.5f};
};