#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/navigate_request.hpp"

using NavigateRequest = transport_interfaces::srv::NavigateRequest;

class NavigationPluginBase {
public:
    virtual ~NavigationPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node);
    virtual void handleNavigation(
        NavigateRequest::Request request,
        NavigateRequest::Response response);

};