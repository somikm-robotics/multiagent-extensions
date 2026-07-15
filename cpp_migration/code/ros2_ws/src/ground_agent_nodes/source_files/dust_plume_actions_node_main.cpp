#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "dust_plume_actions_node.hpp"

int main(int argc, char** argv) {

    auto node = std::make_shared<DustPlumeActionsNode>();
    node->initialise();
    try {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}