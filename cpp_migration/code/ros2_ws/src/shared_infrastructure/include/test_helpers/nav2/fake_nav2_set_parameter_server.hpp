#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"

class FakeSetParameterServer : public rclcpp::Node {
public:
    using SetParameters = rcl_interfaces::srv::SetParameters;
    FakeSetParameterServer();
    
private:
    rclcpp::Service<SetParameters>::SharedPtr fakeService_;

    void handleRequest(std::shared_ptr<SetParameters::Request> request, 
                          std::shared_ptr< SetParameters::Response> response);
};

