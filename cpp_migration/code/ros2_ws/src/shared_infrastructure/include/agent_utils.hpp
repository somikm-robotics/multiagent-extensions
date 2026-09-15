#pragma once

#include <string>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <map>
#include <exception>
#include <cmath>
#include <array>
#include <variant>
#include <boost/process.hpp>
#include <tuple>
#include<optional>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "shared_infrastructure/enums.hpp"
#include "shared_infrastructure/data.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace bp = boost::process;

using PoseStamped = geometry_msgs::msg::PoseStamped;

class AgentUtils {
    
public:
    
    static std::string getCurrentTimeFormatString(std::string format = "");

    static HazardType parseHazardType(const std::string hazardTypeStr);

    static std::string toHazardTypeString(const HazardType hazardType);

    static TaskType parseTaskType(const std::string taskTypeStr);

    static std::string toSeverityLevelStr(SeverityLevel severityLevel);

    static std::string toTaskTypeStr(const TaskType taskType);

    static std::string toOrbitModeStr(const OrbitMode orbitMode);

    static CommandType parseCommandType(const std::string commandTypeStr);

    static float getServiceTimeoutFromConfig();

    static std::string getImageOutputFolderPath();

    static float getStoppingDistanceFromHazardFromConfig();

    static float getNavigationOverrideTimeoutFromConfig();

    static float getAgilexCameraOffsetFromConfig();

    static SimplePose getAgilexBasePose();

    static geometry_msgs::msg::Pose getPose(SimplePose simplePose, float orientationZ, float orientationW);

    static HazardTypePose getCurrentHazardTypePose();
    
    static double calculateOrbitSafetyRadius(const HazardType hazardType, const float hazardDiameter);

    static bool isOrbitClockwise();

    static tf2::Quaternion yawtoQuat(const float yaw);

    static float quatToYaw (const geometry_msgs::msg::Quaternion& quat);

    static float getPoseDistance(const PoseStamped& a, const PoseStamped& b);

    static double normaliseAngle(double angle);

    static double yawFromPose(const geometry_msgs::msg::Pose& p);

    static std::string getCurrentOrbitHazardPlugin();

    static OrbitHazardConfigParams getOrbitHazardParams();

    static DustSensorRelayParams getDustSensorRelayParams();

    static void clearFolder(std::filesystem::path folderPath);

    static std::tuple<bp::child, bp::ipstream> startRosbag(const std::string& outDir);

    static void stopRosbag(boost::process::child& bagProc, bp::ipstream& err);
    
    template<typename... Args>
    static std::string getMessage(Args... args) {
        std::ostringstream ss;
        (ss << ... << args);
        return ss.str();
    }

    template<typename... Args>
    static std::string getFolderPathStr(Args... args) { 

        std::filesystem::path fullPath;

        ((fullPath /= args), ...);

        return fullPath;
    }
private:
    inline static const std::string CONFIG_PATH = R"(/root/ros2_ws/src/shared_infrastructure/config/config.json)";    
    static std::map<HazardType, SimplePose> hazardPoses_;
    
    static nlohmann::json getConfig();
    static nlohmann::json& loadConfig();
    static float getStandOffRadius(const HazardType hazardType);
    static void loadHazardPoses();  

};


