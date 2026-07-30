#include <rclcpp/rclcpp.hpp>

#include "alert_receiver_node.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlertReceiverNode>();
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