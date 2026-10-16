#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "mission_handler_node.hpp"

int main(int argc, char** argv) 
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionHandlerNode>();
	node->initialise();

	RCLCPP_INFO(node->get_logger(), "MissionHandler Node has been started successfully & is ready");

    try {
        rclcpp::spin(node);
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}
