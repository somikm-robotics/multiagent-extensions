#include "mission_overrrider_node.hpp"
#include "agent_utils.hpp"
#include "supervisory_agent_data_provider.hpp"

MissionOverrideNode::MissionOverrideNode() : Node("mission_override_node")
{
   // alertsPub_ = this->create_publisher<transportMsgs::UIAlerts>("/supervisor/status", 10);
    scanIncidentUISrv_ = this->create_service<ScanIncident>("/scan/incident", 
                        [this](const ScanIncident::Request::SharedPtr& request, 
                                ScanIncident::Response::SharedPtr response){
                            this->scanIncident(request, response);
                        });

    overrideMissionUISrv_ = this->create_service<OverrideMission>("/ui/override/mission",
                            [this](const OverrideMission::Request::SharedPtr& request, 
                                        OverrideMission::Response::SharedPtr response){
                                this->overrideMission(request, response);
                            });                        
    overrideMissionClient_ = this->create_client<OverrideMission>("override_mission"); 
    
    hazardDetectedPub_ = this->create_publisher<transportMsgs::HazardPose>("hazard_detected", 10);

    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();
}

void MissionOverrideNode::scanIncident(const ScanIncident::Request::SharedPtr& request, 
                                        ScanIncident::Response::SharedPtr response)
{
    auto locationName = request->location_name;
    RCLCPP_INFO(this->get_logger(), "In Scan incident: %s", locationName.c_str());
 
    auto locationsData = SupervisoryAgentDataProvider::getLocations();
    
    auto itr = locationsData.find(locationName);
    
    if(itr == locationsData.end())
    {
        response->accepted = false;
        response->message = "No such location";
        return;
    }
    
    auto locationshazardTypePoint = itr->second;
    auto locationsPoint = locationshazardTypePoint.hazardPose;
    transportMsgs::HazardPose hazardDetectedMsg;
    hazardDetectedMsg.hazard_type = AgentUtils::toHazardTypeString(locationshazardTypePoint.hazardType);
    hazardDetectedMsg.estimated_hazard_diameter = locationshazardTypePoint.hazardDimeter;
    geometry_msgs::msg::Point centerPoint;
    centerPoint.x = locationsPoint.x;
    centerPoint.y = locationsPoint.y;
    centerPoint.z = 0.5;
    hazardDetectedMsg.hazard_center_point = centerPoint;
    geometry_msgs::msg::Pose hazardPose;
    hazardPose.position.x = locationsPoint.x;
    hazardPose.position.y = locationsPoint.y;
    hazardPose.orientation.w = 1.0;
    hazardDetectedMsg.pose = hazardPose;

    hazardDetectedPub_->publish(hazardDetectedMsg);
    response->accepted = true;
    response->message = "Hazard detected message published";
    RCLCPP_INFO_STREAM(this->get_logger(), "hazard pose: x: " << locationsPoint.x << " y: " << locationsPoint.y );
}

void MissionOverrideNode::overrideMission(const OverrideMission::Request::SharedPtr& request, 
                                            OverrideMission::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "In Override mission: %s Is Initial Inspection: %s", request->command.c_str(),
                                    request->is_initial_inspection ? "yes" : "no");

    RCLCPP_INFO(this->get_logger(), "In override mission timeout configured: %d", serviceTimeout_);
    if(!overrideMissionClient_->wait_for_service(std::chrono::minutes(serviceTimeout_))) {
        auto msg = "Override Mission Service not available";
        RCLCPP_INFO(this->get_logger(), msg);
        response->success = false;
        response->status = msg;
        return;
    }

    auto future = overrideMissionClient_->async_send_request(request);

    if(future.wait_for(std::chrono::minutes(serviceTimeout_)) == std::future_status::timeout) {
        auto msg = "Override Mission Service call timed out";
        RCLCPP_INFO(this->get_logger(), msg);
        response->success = false;
        response->status = msg;
        return;
    }

    auto ssResponse = future.get();
    
    response->success = ssResponse->success;
    response->status = ssResponse->status;

}
