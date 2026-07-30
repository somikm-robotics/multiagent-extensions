#pragma once

#include <string>
#include <map>

#include "shared_infrastructure/data.hpp"

class SupervisoryAgentDataProvider {
public:
    static std::map<std::string, HazardTypePose> getLocations();
private:
    inline static const std::string locationsPath = R"(/root/ros2_ws/src/supervisory_agent/config/hazard_locations.json)";    
    inline static std::map<std::string, HazardTypePose> locationsMap_;
};