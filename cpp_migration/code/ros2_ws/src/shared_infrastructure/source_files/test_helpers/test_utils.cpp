#include "test_helpers/test_utils.hpp"
#include "transport_interfaces/srv/density_regressor.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using DensityRegressor = transport_interfaces::srv::DensityRegressor;

std::unique_ptr<bp::child> TestUtils::createDensityRegressorProcess()
{
    auto densityRegressorServiceProcess = std::make_unique<bp::child>(
                bp::search_path("ros2"),
                bp::args({
                    "run",
                    "ai_python",
                    "density_regressor_node"
                }),
                bp::std_out > stdout,
                bp::std_err > stderr);
            
        auto testNode = std::make_shared<rclcpp::Node>("test_node");
        auto densityRegressorClient = testNode->create_client<DensityRegressor>("density_regressor");

        if(densityRegressorClient->wait_for_service(10s))
            RCLCPP_INFO(testNode->get_logger(), "Density Regressor Service Process started");

        return densityRegressorServiceProcess;
}