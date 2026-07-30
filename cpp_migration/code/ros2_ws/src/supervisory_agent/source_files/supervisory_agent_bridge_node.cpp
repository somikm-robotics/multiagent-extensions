#include "supervisory_agent_bridge_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include "supervisory_agent_data_provider.hpp"
#include "agent_utils.hpp"

SupervisoryAgentBridgeNode::SupervisoryAgentBridgeNode() : Node("supervisory_agent_bridge_node")
{
    statusSub_ = this->create_subscription<transportMsgs::UIAlerts>("/supervisor/alert", 10,
                    [this](const transportMsgs::UIAlerts& uiAlerts){
                        this->onAlert(uiAlerts);
                    });   
    scanIncidentClient_ = this->create_client<ScanIncident>("/scan/incident");
    overrideMissionClient_ = this->create_client<OverrideMission>("/ui/override/mission");

    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
    RCLCPP_INFO(this->get_logger(), "SupervisorBridgeNode started");
}

std::string SupervisoryAgentBridgeNode::waitForStatusMessage()
{
   // RCLCPP_INFO(this->get_logger(), "In Wait for status message");
    // condition_variable::wait requires unique lock. 
    // wait needs to unlock and relock the mutex while waiting
    std::unique_lock<std::mutex> lock(statusQueueMutex_);
    statusQueueCv_.wait(lock, [this] { return !statusMessageQueue_.empty();});
    std::string statusMsg = statusMessageQueue_.front();
    statusMessageQueue_.pop_front();
    //RCLCPP_INFO(this->get_logger(), "In Wait for status message - status- %s", statusMsg.c_str());
    return statusMsg;
}

std::string SupervisoryAgentBridgeNode::waitForHistoryMessage()
{
    //RCLCPP_INFO(this->get_logger(), "In Wait for history message");
    std::unique_lock<std::mutex> lock(historyQueueMutex_);
    historyQueueCv_.wait(lock, [this] { return !historyMessageQueue_.empty();});
    std::string statusMsg = historyMessageQueue_.front();
    historyMessageQueue_.pop_front();
    //RCLCPP_INFO(this->get_logger(), "In Wait for history message - status- %s", statusMsg.c_str());
    return statusMsg;
}

std::pair<bool, std::string> SupervisoryAgentBridgeNode::overrideMission(const std::string &label, const std::string& action)
{
    //RCLCPP_INFO(this->get_logger(), "In override mission - bridge node");
    if(!overrideMissionClient_->wait_for_service(std::chrono::minutes(serviceTimeout_))) {
        auto msg = "Override Mission Service not available";
        RCLCPP_INFO(this->get_logger(), msg);
        return {false, msg};
    }

    auto request = std::make_shared<OverrideMission::Request>();
    request->command = action;
    request->is_initial_inspection = label.rfind("inspection", 0) == 0;

    auto future = overrideMissionClient_->async_send_request(request);

    if(future.wait_for(std::chrono::minutes(serviceTimeout_)) == std::future_status::timeout) {
        auto msg = "Override Mission Service call timed out";
        RCLCPP_INFO(this->get_logger(), msg);
        return {false, msg};
    }

    auto response = future.get();
   // RCLCPP_INFO(this->get_logger(), "In override mission - bridge node - response message: %s", response->status.c_str());
    return { response->success, response->status};
    
}

std::pair<bool, std::string> SupervisoryAgentBridgeNode::scanIncident(const std::string &locationName)
{
   // RCLCPP_INFO(this->get_logger(), "In Scan Incident - bridge node");
    if(!scanIncidentClient_->wait_for_service(std::chrono::minutes(serviceTimeout_))) {
        auto msg = "Scan Incident Service not available";
        RCLCPP_INFO(this->get_logger(), msg);
        return {false, msg};
    }

    auto request = std::make_shared<ScanIncident::Request>();
    request->location_name = locationName;

    auto future = scanIncidentClient_->async_send_request(request);

    if(future.wait_for(std::chrono::minutes(serviceTimeout_)) == std::future_status::timeout) {
        auto msg = "Scan Incident Service call timed out";
        RCLCPP_INFO(this->get_logger(), msg);
        return {false, msg};
    }

    auto response = future.get();
    //RCLCPP_INFO(this->get_logger(), "In Scan Incident - bridge node - response message: %s", response->message.c_str());
    return { response->accepted, response->message};

}

void SupervisoryAgentBridgeNode::onAlert(transportMsgs::UIAlerts alertsMsg)
{
    //RCLCPP_INFO(this->get_logger(), "In on alerts - bridge node");
    if(alertsMsg.is_status) {
        std::lock_guard<std::mutex> lock(statusQueueMutex_);
        statusMessageQueue_.push_back(alertsMsg.message);
        statusQueueCv_.notify_all();
    }
    nlohmann::json jObj;
    jObj["msg"] = alertsMsg.message;
    jObj["inspection-approve"] = alertsMsg.enable_inspection;
    jObj["inspection-cancel"] = alertsMsg.enable_inspection;
    jObj["analysis-approve"] = alertsMsg.enable_analysis;
    jObj["analysis-cancel"] = alertsMsg.enable_analysis;
    jObj["scan"] = alertsMsg.enable_scan;
    
    
    std::lock_guard<std::mutex> lock(historyQueueMutex_);
    historyMessageQueue_.push_back(jObj.dump());
    historyQueueCv_.notify_all();

}

std::string SupervisoryAgentBridgeNode::getLocationsJson()
{
    //RCLCPP_INFO(this->get_logger(), "Getting locations in bridge");
    std::vector<std::string> locationNames;
    auto locations = SupervisoryAgentDataProvider::getLocations();
    //RCLCPP_INFO(this->get_logger(), "Locations count: %zu", locations.size());
    for(const auto& [k, v]: locations) {
        locationNames.push_back(k);
    }    
    nlohmann::json locationNamesJson = locationNames;
    return locationNamesJson.dump();
}

/*
std::string SupervisoryAgentBridgeNode::waitForMessage(bool isStatus)
{
    RCLCPP_INFO(this->get_logger(), "In Wait for message");
    auto& mutex = isStatus ? statusQueueMutex_ : historyQueueMutex_;
    auto messageQueue = isStatus ? statusMessageQueue_ : historyMessageQueue_;
    auto& queueCv = isStatus ? statusQueueCv_ : historyQueueCv_;
    // condition_variable::wait requires unique lock. 
    // wait needs to unlock and relock the mutex while waiting
    std::unique_lock<std::mutex> lock(mutex);
    queueCv.wait(lock, [this, messageQueue] { return !messageQueue.empty();});
    std::string statusMsg = messageQueue.front();
    messageQueue.pop_front();
    RCLCPP_INFO(this->get_logger(), "In Wait for message - status- %s", statusMsg.c_str());
    return statusMsg;
}
*/
