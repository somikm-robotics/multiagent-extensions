#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <memory>

#include "navigation_plugin/navigation_plugin_base.hpp"
#include "transport_interfaces/msg/navigation_result.hpp"

#include "shared_infrastructure/enums.hpp"

namespace transportMsgs = transport_interfaces::msg;

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class Nav2NavigationPlugin : public NavigationPluginBase {

private:
    // publishers
    rclcpp::Publisher<transportMsgs::NavigationResult>::SharedPtr reachedTargetPub_;

    // Services
    rclcpp::Service<Navigate>::SharedPtr navigateService_;    

    // Nav2 client 
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr followWaypointsClient_;
    
    rclcpp::Node::SharedPtr node_;

    // future handlers
    void followWaypointsGoalResponse(GoalHandle::SharedPtr goal_handle);
    void followWaypointsResult(const GoalHandle::WrappedResult& result);

    // variables
    float serviceTimeout_;
    std::string hazardType_;
    

public:
    Nav2NavigationPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    void handleNavigation(std::shared_ptr<Navigate::Request> request) override;

    ~Nav2NavigationPlugin() = default;
};