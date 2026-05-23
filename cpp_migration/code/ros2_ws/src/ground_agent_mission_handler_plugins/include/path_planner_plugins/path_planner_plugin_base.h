#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/request_path_plan.hpp"

using RequestPathPlan = transport_interfaces::srv::RequestPathPlan;

class PathPlannerPluginBase {
public:
    
    virtual void initialise(const rclcpp::Node::SharedPtr & node);
    virtual void plan_path(
        RequestPathPlan::Request request,
        RequestPathPlan::Response response);

    virtual ~PathPlannerPluginBase() = default;

};