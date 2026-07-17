#include "path_planner_plugin/path_planner_plugin.hpp"
#include "agent_utils.hpp"#include "path_planner_plugin.hpp"

PathPlannerPlugin::PathPlannerPlugin(std::string pluginName) 
            : logger_{rclcpp::get_logger(pluginName)}
{
}

void PathPlannerPlugin::coreInitialise(const rclcpp::Node::SharedPtr &node)
{
    node_ = node;
    requestPathPlanActionClient_ = rclcpp_action::create_client<RequestPathPlan>(node, "request_path_plan");
    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
    RCLCPP_INFO(logger_, "Path Planner Plugin initialised");
}
