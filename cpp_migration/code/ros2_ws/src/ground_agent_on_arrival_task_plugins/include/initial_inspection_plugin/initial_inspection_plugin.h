#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/initial_inspection.hpp"

namespace transportMsgs = transport_interfaces::msg;
using InitialInspection = transport_interfaces::srv::InitialInspection;

class InitialInspectionPlugin : public InitialInspectionPluginBase {

private:
    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twistPub;
    rclcpp::Publisher<transportMsgs::MissionCompleted>::SharedPtr _inspectionCompletedPub;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub;

    // Services
    rclcpp::Service<InitialInspection>::SharedPtr _initialInspectionService;    

    inline static const std::string CAMERA_TOPIC 
        {"/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image"};
    
    rclcpp::Node::SharedPtr _node;

    // methods
    void imageCallback(const sensor_msgs::msg::Image& image);
    void odomCallback(const nav_msgs::msg::Odometry& image);
    void spinPublish();
    void saveImage(const sensor_msgs::msg::Image& image);
    void stopSpin();

public:
    InitialInspectionPlugin();

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    void perform_inspection(
        InitialInspection::Request request,
        InitialInspection::Response response) override;

    ~InitialInspectionPlugin() = default;
};