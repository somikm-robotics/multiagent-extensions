#include "twist_relay_node.hpp"

TwistRelayNode::TwistRelayNode() : Node("twist_relay_node")
{
    twistPublisher_ = this->create_publisher<Twist>("/diff_drive_base_controller/cmd_vel_unstamped", 10);
    twistSubscription_ = this->create_subscription<Twist>("/cmd_vel", 10, 
                            [this](const Twist& twistMsg){
                                this->relayCmd(twistMsg);
                            });

}

void TwistRelayNode::relayCmd(const Twist& twistMsg)
{
    twistPublisher_->publish(twistMsg);
}


int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelayNode>();
    try 
    {
        rclcpp::spin(node);
    } catch(const std::exception &ex) 
    {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    
    rclcpp::shutdown();

    return 0;
}