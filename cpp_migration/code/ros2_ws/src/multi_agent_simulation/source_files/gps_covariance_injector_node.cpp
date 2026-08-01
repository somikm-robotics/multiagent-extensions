#include "gps_covariance_injector_node.hpp"

GPSCovarianceInjectorNode::GPSCovarianceInjectorNode() : 
                Node("gps_covariance_injector_node")
{
    navSatMsgPublisher_ = this->create_publisher<NavSatFix>("/gps/fix/with_cov", 10);
    navSatMsgSubscriber_ = this->create_subscription<NavSatFix>("/gps/fix", 10, 
                            [this](NavSatFix msg){
                                this->inject(msg);
                            });

}

void GPSCovarianceInjectorNode::inject(NavSatFix& navSatMsg)
{
    auto var_xy = 3.0 * 3.0;  // 9
    auto var_z = 6.0 * 6.0; // 36;

    navSatMsg.position_covariance = {
        var_xy, 0.0, 0.0,
        0.0, var_xy, 0.0,
        0.0, 0.0, var_z
    };
    navSatMsg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    navSatMsgPublisher_->publish(navSatMsg);
}


int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSCovarianceInjectorNode>();
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