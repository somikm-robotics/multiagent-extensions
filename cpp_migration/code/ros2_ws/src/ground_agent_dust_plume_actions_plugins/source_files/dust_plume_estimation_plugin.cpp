#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin.hpp"

#include <stdexcept>
#include <functional>
#include "pluginlib/class_list_macros.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include "agent_utils.hpp"

DustPlumeEstimationPlugin::DustPlumeEstimationPlugin() 
    : logger_{rclcpp::get_logger("DustPlumeEstimationPlugin")}
{
}

void DustPlumeEstimationPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
    RCLCPP_INFO(node->get_logger(), "initialising plugin");
    node_ = node;
    completionPub_ = node->create_publisher<transportMsgs::DensityEstimationResult>(
        "estimation_complete", 10);
    
        cameraTopic_ = AgentUtils::getTopicParams().ImageTopicName;

    imageSub_ = node->create_subscription<sensor_msgs::msg::Image>(cameraTopic_, 10, 
        [this](const sensor_msgs::msg::Image::SharedPtr image){
        this->imageCallback(image);
    });

    
    densityRegressorClient_ = node->create_client<DensityRegressor>("density_regressor");
    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();

    RCLCPP_INFO(logger_, "completed plugin initialisation");

} 

void DustPlumeEstimationPlugin::estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> densityRequest)
{
    
    RCLCPP_INFO(logger_, "in estimate plume density");
    
    auto cvImageOpt = getImage();

    if(!cvImageOpt.has_value()) {
        RCLCPP_ERROR(logger_, "got no image for dust plume estimation");
        transportMsgs::DensityEstimationResult result;
        result.success = false;
        result.message = "Some error occured - No image was available for estimation";
        completionPub_->publish(result);
        return;
    }


    RCLCPP_INFO(logger_, "got image: %s", cvImageOpt.has_value() ? "true" : "false");
    auto regressorRequest = std::make_shared<DensityRegressor::Request>();
    regressorRequest->detected_image = cvImageOpt.value();
    
    auto callback = [this](rclcpp::Client<DensityRegressor>::SharedFuture future) {
                                this->processResponse(future);                
                                };

    if(!densityRegressorClient_->wait_for_service(std::chrono::duration<float>(serviceTimeout_))) {
        RCLCPP_INFO(logger_, "density regressor client is not available");                           
    }

    densityRegressorClient_->async_send_request(regressorRequest, callback);
    RCLCPP_INFO(logger_, "sent request");                       
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
    RCLCPP_INFO(logger_, "received response");                       
    transportMsgs::DensityEstimationResult result;
    if(response->success) {
        densityResult_ = response->density;
        auto densityResponse = std::make_shared<PlumeDensity::Response>();
        if(densityResult_.has_value()) {
            auto density = densityResult_.value();
            auto severityLevel = computeSeverityLevel(density);
            auto severityLevelStr = AgentUtils::toSeverityLevelStr(severityLevel);
                    
            auto message = AgentUtils::getMessage("Predicted density: ", density, " Severity Level: ", severityLevelStr);
            RCLCPP_INFO_STREAM(logger_, message);
            result.density = density;
            result.level = severityLevelStr;
            result.success = true;
            result.message = message;
            
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

std::optional<sensor_msgs::msg::Image> DustPlumeEstimationPlugin::getImage()
{
    std::unique_lock<std::mutex> mtx(cvImageMutex_);
    bool available = cvImageCondition_.wait_for(mtx, 
        std::chrono::duration<float>(serviceTimeout_),
        [this](){
        return cvImageOpt_.has_value();
    });

    if(!available) {
        return std::nullopt;
    }

    auto cvImage = std::move(cvImageOpt_).value();
    cvImageOpt_.reset();
    return cvImage;
}

void DustPlumeEstimationPlugin::imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg)
{
    {
        std::lock_guard<std::mutex> mtx(cvImageMutex_);
        cvImageOpt_ = *imageMsg;    
    }

    cvImageCondition_.notify_one();
}


PLUGINLIB_EXPORT_CLASS(DustPlumeEstimationPlugin, DustPlumeEstimationPluginBase)