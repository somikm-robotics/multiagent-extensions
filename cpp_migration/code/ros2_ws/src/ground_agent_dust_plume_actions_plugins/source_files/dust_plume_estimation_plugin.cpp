#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin.hpp"

#include <stdexcept>
#include <functional>
#include "pluginlib/class_list_macros.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include "agent_utils.hpp"

void DustPlumeEstimationPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
    RCLCPP_INFO(node->get_logger(), "initialising plugin");
    node_ = node;
    completionPub_ = node->create_publisher<transportMsgs::DensityEstimationResult>(
        "estimation_complete", 10);

    imageSub_ = node->create_subscription<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10, 
        [this](const sensor_msgs::msg::Image::SharedPtr image){
        this->imageCallback(image);
    });

   // RCLCPP_INFO(node_->get_logger(), "created image callback");
    
    densityRegressorClient_ = node_->create_client<DensityRegressor>("density_regressor");
    firstImagePopulated_ = false;
    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();

    RCLCPP_INFO(node_->get_logger(), "completed plugin initialisation");

} 

void DustPlumeEstimationPlugin::estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> densityRequest)
{
    RCLCPP_INFO(node_->get_logger(), "in estimate plume density");
    while(!cvImageOpt_.has_value()){
        // nothing to do
       // RCLCPP_INFO(node_->get_logger(), "no image yet");
    }

    RCLCPP_INFO(node_->get_logger(), "got image: %s", cvImageOpt_.has_value() ? "true" : "false");
    auto regressorRequest = std::make_shared<DensityRegressor::Request>();
    regressorRequest->detected_image = cvImageOpt_.value();
    
    auto callback = [this](rclcpp::Client<DensityRegressor>::SharedFuture future) {
                                this->processResponse(future);                
                                };

    if(!densityRegressorClient_->wait_for_service(std::chrono::duration<float>(serviceTimeout_))) {
        RCLCPP_INFO(node_->get_logger(), "density regressor client is not available");                           
    }

    densityRegressorClient_->async_send_request(regressorRequest, callback);
    RCLCPP_INFO(node_->get_logger(), "sent request");                       
}                          

SeverityLevel DustPlumeEstimationPlugin::computeSeverityLevel(float density)
{
    SeverityLevel level;
    if(density < 0.2)
        level = SeverityLevel::LOW;
    else if(density < 0.4) 
        level = SeverityLevel::MODERATE;
    else if(density < 0.6) 
        level = SeverityLevel::HIGH;
    else if(density < 0.8)
        level = SeverityLevel::VERY_HIGH;
    else
        level = SeverityLevel::SEVERE;
    
    return level;
}

void DustPlumeEstimationPlugin::processResponse(rclcpp::Client<DensityRegressor>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "received response");                       
    transportMsgs::DensityEstimationResult result;
    if(response->success) {
        densityResult_ = response->density;
        auto densityResponse = std::make_shared<PlumeDensity::Response>();
        if(densityResult_.has_value()) {
            auto density = densityResult_.value();
            auto severityLevel = computeSeverityLevel(density);
            auto severityLevelStr = AgentUtils::toSeverityLevelStr(severityLevel);
                    
            auto message = AgentUtils::getMessage("Predicted density: ", density, " Severity Level", severityLevelStr);
            RCLCPP_INFO_STREAM(node_->get_logger(), message);
            result.density = density;
            result.level = severityLevelStr;
            result.success = true;
            
        } else {
            result.success = false;
            result.message = "Some error occured - Density result returned has no value";
        }
    }
    else {
        result.success = false;
        result.message = "Some error occured - Density regressor did not return as successful response";
    }
        
    completionPub_->publish(result);
}

void DustPlumeEstimationPlugin::imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg)
{
 //   RCLCPP_INFO(node_->get_logger(), "in image callback");
    cvImageOpt_ = *imageMsg;
    if(!firstImagePopulated_)
        firstImagePopulated_ = true;
}


PLUGINLIB_EXPORT_CLASS(DustPlumeEstimationPlugin, DustPlumeEstimationPluginBase)