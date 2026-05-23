
#include "dust_plume_density_estimation_node.h"
#include <stdexcept>
#include <functional>

DustPlumeDensityEstimationNode::DustPlumeDensityEstimationNode() : Node ("dust_plume_density_estimation_node") {

        _completionPub = this->create_publisher<transportMsgs::DensityEstimationResult>(
        "estimation_complete", 10);

        // std::forward has been kept intentionally for now. 
        // This will be reviewed during implementation phase
        _imageSub = this->create_subscription<sensor_msgs::msg::Image>(
            CAMERA_TOPIC, 10, [this](const sensor_msgs::msg::Image& image){
                this->imageCallback(std::forward<decltype(image)>(image));
            });
            
        _plumeDensityService = this->create_service<PlumeDensity>(
            "plume_density", 
                [this] (std::shared_ptr<PlumeDensity::Request> request, 
                    std::shared_ptr<PlumeDensity::Response> response) {
                    this->estimatePlumeDensity(
                        std::forward<decltype(request)>(request), 
                        std::forward<decltype(response)>(response)
                    );        
                } );
}

void DustPlumeDensityEstimationNode::estimatePlumeDensity(
    std::shared_ptr<PlumeDensity::Request> req, 
    std::shared_ptr<PlumeDensity::Response> response) {

        transportMsgs::DensityEstimationResult message;
        message.density = 0.5f;
        this->_completionPub->publish(message);

}

DustPlumeDensityEstimationNode::~DustPlumeDensityEstimationNode() {

}

void DustPlumeDensityEstimationNode::imageCallback(const sensor_msgs::msg::Image& img) {

}

SeverityLevel& DustPlumeDensityEstimationNode::computeSeverityLevel(double density) {
     throw std::logic_error("Not yet Implemented");
}

int main(int argc, char** argv) {

    auto node = std::make_shared<DustPlumeDensityEstimationNode>();

    try {
        rclcpp::spin(node);
    } catch(const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", ex.what());
    }
    rclcpp::shutdown();
    
    return 0;
}