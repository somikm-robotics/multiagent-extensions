#include "supervisory_agent_bridge_node.hpp"
#include "supervisory_agent_http_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <thread>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto bridgeNode = std::make_shared<SupervisoryAgentBridgeNode>();
    
    
    std::string webRoot = ament_index_cpp::get_package_share_directory("supervisory_agent") + "/web_ui";
    auto httpServer = std::make_shared<SupervisoryAgentHttpServer>(webRoot);

    //providers
    httpServer->setStatusMessageProvider([&bridgeNode](){
        return bridgeNode->waitForStatusMessage();
    });

    httpServer->setHistoryMessageProvider([&bridgeNode](){
        return bridgeNode->waitForHistoryMessage();
    });

    httpServer->setDataProvider([&bridgeNode](){
        return bridgeNode->getLocationsJson();
    });

    httpServer->setLogger([&bridgeNode](const std::string& msg){
        RCLCPP_INFO(bridgeNode->get_logger(), "%s", msg.c_str());
    });

    // service handlers
    httpServer->setOverrideMissionHandler([&bridgeNode](const std::string& label, 
            const std::string& action){
        return bridgeNode->overrideMission(label, action);
    });

    httpServer->setScanIncidentHandler([&bridgeNode](const std::string& locationName){
        return bridgeNode->scanIncident(locationName);
    });

    // Spin ROS2 on its own thread
    std::thread rosThread([&bridgeNode]() {
        rclcpp::spin(bridgeNode);
    });

    RCLCPP_INFO(bridgeNode->get_logger(), "HTTP server starting on http://0.0.0.0:8000");
    httpServer->start("0.0.0.0", 8000);

    rclcpp::shutdown();
    rosThread.join();
    return 0;
}