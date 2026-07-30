#include "supervisory_agent_data_provider.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

std::map<std::string, HazardTypePose> SupervisoryAgentDataProvider::getLocations()
{
    //auto logger = rclcpp::get_logger("supervisory_agent_data_provider");
    //RCLCPP_INFO(logger,  "In get locations in data provder- %s", locationsPath.c_str());
    if(!locationsMap_.empty())
        return locationsMap_;

    std::ifstream file(locationsPath);
    
    nlohmann::json locationsData;
    file >> locationsData;
    if(locationsData.contains("Locations")){
        // RCLCPP_INFO(logger,  "In get locations - contains Locations - %s", 
        //                    (locationsData.contains("Locations") ? "yes" : "no"));
       // RCLCPP_INFO(logger, "locations json: %s", locationsData.dump().c_str());

        locationsMap_ = locationsData["Locations"].get<std::map<std::string, HazardTypePose>>();
    }

    //RCLCPP_INFO(logger,  "In get locations in data provder- count %zu", locationsMap_.size());
    return locationsMap_;
}