#pragma once

#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "sensor_msgs/msg/image.hpp"

#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/srv/density_regressor.hpp"

#include "shared_infrastructure/enums.hpp"
#include "dust_plume_estimation_plugin_base.hpp"

using DensityRegressor = transport_interfaces::srv::DensityRegressor;

namespace transportMsgs = transport_interfaces::msg;


class DustPlumeEstimationPlugin : public DustPlumeEstimationPluginBase {

public:
    DustPlumeEstimationPlugin();

    void initialise(const rclcpp::Node::SharedPtr & node) override;

    void estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> request) override;
    
    ~DustPlumeEstimationPlugin() = default;

private:

    // Publishers
    rclcpp::Publisher<transportMsgs::DensityEstimationResult>::SharedPtr completionPub_;
    
    // subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;

    // client
    rclcpp::Client<DensityRegressor>::SharedPtr densityRegressorClient_;

    //callback
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg);

    //variables
    rclcpp::Node::WeakPtr node_;
    rclcpp::Logger logger_;
    std::optional<sensor_msgs::msg::Image> cvImageOpt_;
    std::mutex cvImageMutex_;
    std::condition_variable cvImageCondition_;
    //bool firstImagePopulated_;
    std::optional<float> densityResult_;
    float serviceTimeout_;
    std::string cameraTopic_;
        
    //methods
    SeverityLevel computeSeverityLevel(float density);
    void processResponse(rclcpp::Client<DensityRegressor>::SharedFuture future);
    std::optional<sensor_msgs::msg::Image> getImage();
};