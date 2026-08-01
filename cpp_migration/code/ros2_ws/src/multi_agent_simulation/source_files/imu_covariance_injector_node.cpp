#include "imu_covariance_injector_node.hpp"

ImuCovarianceInjectorNode::ImuCovarianceInjectorNode() : 
                Node("imu_covariance_injector_node")
{
    imuPublisher_ = this->create_publisher<Imu>("/agilex/imu/data/with_cov", 10);
    imuSubcription_ = this->create_subscription<Imu>("/agilex/imu/data", 10, 
                        [this](Imu imuMsg){
                            this->imuCallback(imuMsg);
                        });
}

void ImuCovarianceInjectorNode::imuCallback(Imu& imuMsg)
{
    // diagonal orientation covariance: 0.001 rad² ≈ 0.06°
    imuMsg.orientation_covariance = {1e-3, 0, 0,
                                     0, 1e-3, 0,
                                     0,  0,  1e-3};
    //angular velocity – give Z a finite variance; X,Y small but non-zero                                    
    imuMsg.angular_velocity_covariance = {1e-6, 0.0, 0.0,
                                          0.0,  1e-6, 0.0,
                                          0.0,   0.0,  1e-4};    //← we fuse ωz, so must be >0
    //# linear acceleration (not fused right now, but safe to fill)
    imuMsg.linear_acceleration_covariance = {1e-3, 0.0, 0.0,
                                             0.0,  1e-3, 0.0,
                                             0.0,  0.0,  1e-2};                                           

    imuPublisher_->publish(imuMsg);                                             
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuCovarianceInjectorNode>();
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