#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/srv/initial_inspection.hpp"

using InitialInspection = transport_interfaces::srv::InitialInspection;

class InitialInspectionPluginBase {
public:
    virtual ~InitialInspectionPluginBase() = default;
    virtual void initialise(const rclcpp::Node::SharedPtr & node);
    virtual void perform_inspection(
        InitialInspection::Request request,
        InitialInspection::Response response);

};