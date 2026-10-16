#include "shared_infrastructure/data.hpp"
#include "agent_utils.hpp"
#include <rclcpp/rclcpp.hpp>

void from_json(const nlohmann::json &config, SimplePose& pose)
{
    config.at("x").get_to(pose.x);
    config.at("y").get_to(pose.y);
    if(config.contains("z"))
        config.at("z").get_to(pose.z);
    else
        pose.z = 0.0f;
    if(config.contains("yaw"))        
        config.at("yaw").get_to(pose.yaw);
    else 
        pose.yaw = 1.0f;
}

void from_json(const nlohmann::json &config, HazardType& hazardType)
{
    const auto value = config.get<std::string>();
    //auto logger = rclcpp::get_logger("data");
    //RCLCPP_INFO(logger, "Data: hazard type %s", value.c_str());
    hazardType = AgentUtils::parseHazardType(value);
}
void from_json(const nlohmann::json &config, HazardTypePose &hazardTypePose)
{
    config.at("hazard").get_to(hazardTypePose.hazardType);
    //auto logger = rclcpp::get_logger("data");
    //RCLCPP_INFO(logger, "Data: hazard type pose %s", config.at("hazard").get<std::string>().c_str());
    config.at("x").get_to(hazardTypePose.hazardPose.x);
    config.at("y").get_to(hazardTypePose.hazardPose.y);
    if(config.contains("z"))
        config.at("z").get_to(hazardTypePose.hazardPose.z);
    else
        hazardTypePose.hazardPose.z = 0.0f;
    if(config.contains("yaw"))        
        config.at("yaw").get_to(hazardTypePose.hazardPose.yaw);
    else 
        hazardTypePose.hazardPose.yaw = 1.0f;
    if(config.contains("diameter")) 
        config.at("diameter").get_to(hazardTypePose.hazardDimeter);
    else
        hazardTypePose.hazardDimeter = 0.0f;
}