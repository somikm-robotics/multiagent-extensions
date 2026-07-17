#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/initial_inspection.hpp"

#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>

#include "initial_inspection_plugin/initial_inspection_plugin_base.hpp"

#include "shared_infrastructure/enums.hpp"


namespace transportMsgs = transport_interfaces::msg;
using InitialInspection = transport_interfaces::srv::InitialInspection;

class InitialInspectionPlugin : public InitialInspectionPluginBase {

public:
    InitialInspectionPlugin();

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    std::shared_ptr<InitialInspection::Response> performInspection(
        std::shared_ptr<InitialInspection::Request> request) override;

    ~InitialInspectionPlugin() override = default;    


private:

    // constants
    inline static const std::string kCameraTopic =
        R"(/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image)";
    
    inline static const std::string kCmdVelTopic = "/cmd_vel";
    inline static const int kNImages = 10;
    inline static const float kSpinSpeed = 0.5f;
    inline static const double kAnglePerImage = 30.0 * M_PI / 180.0;
    inline static const float kPhotoRate = 2.0f;
    inline static const std::string kImagesOutDir  = "initial_inspection";
    inline static const int kPubFrequencyHz = 10;
    inline static const std::string kImageFormat = "png";

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
    rclcpp::Publisher<transportMsgs::MissionCompleted>::SharedPtr inspectionCompletedPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr spinPub_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    // Services
    rclcpp::Service<InitialInspection>::SharedPtr initialInspectionService_;    

   // instance variables
    rclcpp::Node::WeakPtr node_;
    rclcpp::Logger logger_;
    HazardType hazardType_;
    bool busy_;
    bool spinActive_;
    int imagesCaptured_;
    std::optional<double> lastYaw_;
    rclcpp::TimerBase::SharedPtr spinTimer_;
    std::optional<double> currentYaw_;
    geometry_msgs::msg::Twist twist_;
    bool hasSpinPub_;
    std::string outDir_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void spinPublish();
    void saveImage(const cv::Mat& image);
    void stopSpin();
};