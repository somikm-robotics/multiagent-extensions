#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <boost/process.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pluginlib/class_list_macros.hpp"

#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"
#include "shared_infrastructure/data.hpp"
#include "shared_infrastructure/enums.hpp"

#include "orbit_hazard_plugin/orbit_hazard_manager_plugin_base.hpp"

namespace transportMsgs = transport_interfaces::msg;
namespace bp = boost::process;

using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class OrbitHazardManagerPlugin : public OrbitHazardManagerPluginBase {

private:

protected:
    // publishers
    rclcpp::Publisher<transportMsgs::MissionCompleted>::SharedPtr orbitCompletedPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;

    // subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    // variables
    rclcpp::Node::SharedPtr node_;

    OrbitHazardConfigParams configParams_;

    float serviceTimeout_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    rclcpp::TimerBase::SharedPtr tfIssuesTimer_;

    float hazardCenterPointX_;
    float hazardCenterPointY_;
    float estimatedHazardDiameter_;
    HazardType hazardType_;

    float stagingX_;
    float stagingY_;
    float stagingYaw_;

     // variables
    float currentX_;
    float currentY_;
    std::optional<float> currentYaw_;

    std::string bagDir_;
    bp::child bagProc_;
    bp::ipstream bagErr_;

    float radius_;
    std::optional<float> previousBearing_;
    OrbitMode orbitMode_;
    int tick_;
    rclcpp::TimerBase::SharedPtr stagingTimer_;

    bool inProgress_;
    bool stagingCompleted_;
    bool orbitCompleted_;

    // callbacks
    void odomCallback(const nav_msgs::msg::Odometry& msg);

    //methods
    void navigateToStagingPose(std::string rosBagDir);
    std::tuple<float, float> calculateRadiusAndBearing();
   // std::optional<DistanceWithCoOrds> calculateStagingDistance();
   // std::optional<SimplePose> computeStagingPose();
    std::optional<geometry_msgs::msg::TransformStamped> lookupTransform();
    void populateOrientation(geometry_msgs::msg::PoseStamped& pose);
    void shutdownGracefully();
    void publishOrbitCompleted(bool success);
    void restart();
    
    void setFromRequestParams(std::shared_ptr<OrbitHazard::Request> request);
    void stagingLoop();
    void doRadialIn(const float& radialError, const float& yawError);
    void doRadialOut(const float& radialError, const float& yawError);
    void doTangentAlign(const float& bearing, const float& yawError);
    std::string getLogMessage(float yawError, geometry_msgs::msg::Twist twist, std::optional<float> radialError = std::nullopt);

    virtual void startOrbit() = 0;
   
public:
    OrbitHazardManagerPlugin()=default;

    virtual void initialise(const rclcpp::Node::SharedPtr & node) override;

    virtual void doHazardOrbit(
        std::shared_ptr<OrbitHazard::Request> request) override = 0;

    ~OrbitHazardManagerPlugin() override = default;
};