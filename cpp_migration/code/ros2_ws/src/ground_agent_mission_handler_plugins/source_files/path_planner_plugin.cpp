#include "path_planner_plugin/path_planner_plugin.hpp"
#include "agent_utils.hpp"

void PathPlannerPlugin::coreInitialise(const rclcpp::Node::SharedPtr& node)
{
    node_ = node;
    requestPathPlanActionClient_ = rclcpp_action::create_client<RequestPathPlan>(node, "request_path_plan");
    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
    RCLCPP_INFO(node_->get_logger(), "Path Planner Plugin initialised");
}
