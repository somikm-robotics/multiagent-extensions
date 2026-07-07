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

private:

    inline static const std::string CAMERA_TOPIC {"/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image"};

    // Publishers
    rclcpp::Publisher<transportMsgs::DensityEstimationResult>::SharedPtr completionPub_;
    
    // subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;

    // client
    rclcpp::Client<DensityRegressor>::SharedPtr densityRegressorClient_;

    //callback
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg);
    //rclcpp::Client<DensityRegressor>::SharedFuture future
    //std::optional<float> getDensityResult(rclcpp::Client<DensityRegressor>::SharedFuture future);

    //variables
    rclcpp::Node::SharedPtr node_;
    std::optional<sensor_msgs::msg::Image> cvImageOpt_;
    bool firstImagePopulated_;
    std::optional<float> densityResult_;
    float serviceTimeout_;
        
    //methods
    SeverityLevel computeSeverityLevel(float density);
    void processResponse(rclcpp::Client<DensityRegressor>::SharedFuture future);

public:
    DustPlumeEstimationPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override;

    void estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> request) override;
    
    ~DustPlumeEstimationPlugin() = default;
};