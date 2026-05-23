#include "rclcpp/rclcpp.hpp"
#include <memory>


#include "transport_interfaces/srv/request_path_plan.hpp"
#include "transport_interfaces/srv/path_plan_result.hpp"

using RequestPathPlan = transport_interfaces::srv::RequestPathPlan;
using PathPlanResult = transport_interfaces::srv::PathPlanResult;

class PathPlannerPlugin : public PathPlannerPluginBase {

private:

    // Services
    rclcpp::Service<RequestPathPlan>::SharedPtr _pathPlannerService;  
    
    // client
    rclcpp::Client<PathPlanResult>::SharedPtr _pathPlanerResultClient;

    rclcpp::Node::SharedPtr _node;
    
   
public:
    PathPlannerPlugin()=default;

    void initialise(const rclcpp::Node::SharedPtr & node) override;

    ~PathPlannerPlugin() = default;
};