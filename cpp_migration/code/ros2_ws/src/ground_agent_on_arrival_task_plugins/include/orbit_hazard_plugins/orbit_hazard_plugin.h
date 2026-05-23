#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"

namespace transportMsgs = transport_interfaces::msg;
using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class OrbitHazardPlugin : public OrbitHazardPluginBase {

private:
    // publishers
    rclcpp::Publisher<transportMsgs::MissionCompleted>::SharedPtr _orbitCompletedPub;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub;

    // Services
    rclcpp::Service<OrbitHazard>::SharedPtr _orbitHazardService;    

    rclcpp::Node::SharedPtr _node;
    
    void odomCallback(const nav_msgs::msg::Odometry& image);
    void setRadius(const float& estimatedHazardDiameter);
    
   
public:
    OrbitHazardPlugin();

    void initialise(const rclcpp::Node::SharedPtr & node) override;

    ~OrbitHazardPlugin() = default;
};