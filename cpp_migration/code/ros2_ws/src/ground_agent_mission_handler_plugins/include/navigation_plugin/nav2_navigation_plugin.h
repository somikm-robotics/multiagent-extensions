#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include <memory>

#include "transport_interfaces/msg/navigation_result.hpp"

namespace transportMsgs = transport_interfaces::msg;
using NavigateRequest = transport_interfaces::srv::NavigateRequest;

class Nav2NavigationPlugin : public NavigationPluginBase {

private:
    // publishers
    rclcpp::Publisher<transportMsgs::NavigationResult>::SharedPtr _navigationResultPub;

    // Services
    rclcpp::Service<NavigateRequest>::SharedPtr _navigateService;    

    // Nav2 client 
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr _computePathToPoseClient;
    
    rclcpp::Node::SharedPtr _node;

    // future handlers
    followPathGoalResponse(
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalFuture future);
    followPathResult(
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::ResultFuture future);
    
public:
    Nav2NavigationPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override;
    void handleNavigation(
        NavigateRequest::Request request,
        NavigateRequest::Response response);

    ~Nav2NavigationPlugin() = default;
};