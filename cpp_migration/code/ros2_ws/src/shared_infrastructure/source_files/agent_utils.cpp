#include "agent_utils.hpp"
#include <stdexcept>
#include <iostream>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::map<HazardType, SimplePose> AgentUtils::hazardPoses_;

std::string AgentUtils::getCurrentTimeFormatString(std::string format)
{
    if(format == "")
        format = "%Y%m%d_%H%M%S";

    std::time_t t = std::time(nullptr);
    std::tm& tm  = *std::localtime(&t);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), format.c_str(), &tm);
    return std::string(buffer);
}

HazardType AgentUtils::parseHazardType(const std::string hazardTypeStr)
{
    return hazardTypeStr == "DUST_PLUME" ? HazardType::DUST_PLUME : 
                    (hazardTypeStr == "FIBROUS_HAZARD" ? HazardType::FIBROUS_HAZARD : HazardType::NONE);
}

std::string AgentUtils::toHazardTypeString(const HazardType hazardType)
{
    std::string hazardTypeStr;
    switch (hazardType)
    {
        case HazardType::DUST_PLUME:
            hazardTypeStr = "DUST_PLUME";
            break;
        case HazardType::FIBROUS_HAZARD:
            hazardTypeStr = "FIBROUS_HAZARD";
            break;
    
        default:
            hazardTypeStr = "NONE";
            break;
    }

    return hazardTypeStr;
}

CommandType AgentUtils::parseCommandType(const std::string commandTypeStr)
{
    return commandTypeStr == "APPROVED" ? CommandType::APPROVED : CommandType::CANCELLED;
}

TaskType AgentUtils::parseTaskType(const std::string taskTypeStr)
{
    return taskTypeStr == "INITIAL_INSPECTION" ? TaskType::INITIAL_INSPECTION 
        : (taskTypeStr == "DENSITY_ESTIMATION" ? TaskType::DENSITY_ESTIMATION 
        : (taskTypeStr == "ASBESTOS_ANALYSIS" ? TaskType::ASBESTOS_ANALYSIS : TaskType::NONE));
}

std::string AgentUtils::toSeverityLevelStr(SeverityLevel severityLevel)
{
    std::string severityLevelStr;

    switch (severityLevel)
    {
    case SeverityLevel::LOW:
        severityLevelStr = "Low";
        break;
    case SeverityLevel::MODERATE:
        severityLevelStr = "Moderate";
        break;
    case SeverityLevel::HIGH:
        severityLevelStr = "High";
        break;
    case SeverityLevel::VERY_HIGH:
        severityLevelStr = "Very High";
        break;
    case SeverityLevel::SEVERE:
        severityLevelStr = "Severe";
        break;
    default:
        severityLevelStr = "None";
        break;
    }

    return severityLevelStr;
}

std::string AgentUtils::toTaskTypeStr(const TaskType taskType)
{
    std::string taskTypeStr;
    switch (taskType)
    {
        case TaskType::INITIAL_INSPECTION:
            taskTypeStr = "INITIAL_INSPECTION";
            break;
        case TaskType::ASBESTOS_ANALYSIS:
            taskTypeStr = "ASBESTOS_ANALYSIS";
            break;
        case TaskType::DENSITY_ESTIMATION:
            taskTypeStr = "DENSITY_ESTIMATION";
            break;
        default:
            taskTypeStr = "NONE";
            break;
    }

    return taskTypeStr;
}

std::string AgentUtils::toOrbitModeStr(const OrbitMode orbitMode)
{
    std::string orbitModeStr;
    switch (orbitMode)
    {
        case OrbitMode::RADIAL_IN:
            orbitModeStr = "RADIAL_IN";
            break;
        case OrbitMode::RADIAL_OUT:
            orbitModeStr = "RADIAL_OUT";
            break;
        case OrbitMode::TANGENT_ALIGN:
            orbitModeStr = "TANGENT_ALIGN";
            break;
        default:
            orbitModeStr = "NONE";
            break;
    }

    return orbitModeStr;
}

float AgentUtils::getServiceTimeoutFromConfig()
{
    auto& config = loadConfig();
    return config.value("service_timeout", 10.0);
}

std::string AgentUtils::getImageOutputFolderPath()
{
    auto& config = loadConfig();
    return config.value("image_output_path", "");
}

float AgentUtils::getStoppingDistanceFromHazardFromConfig()
{
    auto& config = loadConfig();
    return config.value("stopping_distance_y_from_hazard", 0.5);
}

float AgentUtils::getNavigationOverrideTimeoutFromConfig()
{
    auto& config = loadConfig();
    return config.value("navigation_override_timeout_in_minutes", 10.0);
}

float AgentUtils::getAgilexCameraOffsetFromConfig()
{
    auto& config = loadConfig();
    return config.value("agilex_camera_offset", 0.5);
}

SimplePose AgentUtils::getAgilexBasePose()
{
    auto& config = loadConfig();
    auto pose = config.value("agilex_base_pose", std::vector<float>{});
    SimplePose simplePose;
    if(pose.size() == 4) {
        simplePose.x = pose[0];
        simplePose.y = pose[1];
        simplePose.z = pose[2];
        simplePose.yaw = pose[3];
    }
    return simplePose;
}

geometry_msgs::msg::Pose AgentUtils::getPose(SimplePose simplePose, float orientationZ, float orientationW)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = simplePose.x;
    pose.position.y = simplePose.y;
    pose.position.z = simplePose.z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = orientationZ;
    pose.orientation.w = orientationW;

    return pose;
}

HazardTypePose AgentUtils::getCurrentHazardTypePose()
{
    auto& config = loadConfig();
    auto hazardTypeStr = config.value("current_hazard_target", "FIBROUS_HAZARD");
    auto parsedHazardType = parseHazardType(hazardTypeStr);
    auto itr = hazardPoses_.find(parsedHazardType);
    auto hazardPoses =  itr != hazardPoses_.end() ? itr->second : SimplePose{};

    HazardTypePose hazardTypePose {
        parsedHazardType,
        hazardPoses
    };

    return hazardTypePose;
}

double AgentUtils::calculateOrbitSafetyRadius(const HazardType hazardType, const float hazardDiameter)
{
    auto& config = loadConfig();
    auto fovDeg = config.value("agilex_camera_fov_deg", 0.0f);
    auto robotLM = config.value("agile_length_m", 0.0f);
    auto robotWM = config.value("agile_width_m", 0.0f);
    auto maginM = config.value("additional_safety_margin", 0.0f);

    double fovRad = fovDeg * M_PI / 180.0;
    auto standoffRadius = getStandOffRadius(hazardType);
    auto rCamera = (hazardDiameter/2) / tan(fovRad / 2);
    auto rRobot = hypot(robotLM/2, robotWM /2) + maginM;
    return std::max({static_cast<double>(standoffRadius), rCamera, rRobot});
}

bool AgentUtils::isOrbitClockwise()
{
    auto& config = loadConfig();
    return config.value("is_orbit_clockwise", true) == true;
}

std::array<float, 4> AgentUtils::yawtoQuat(const float yaw)
{
    std::array<float, 4> quat;
    quat[0] = 0.0;
    quat[1] = 0.0;
    quat[2] = sin(yaw * 0.5);
    quat[3] = cos(yaw * 0.5);

    return quat;
}

float AgentUtils::quatToYaw(const geometry_msgs::msg::Quaternion& quat)
{
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

float AgentUtils::getPoseDistance(const PoseStamped& a, const PoseStamped& b)
{
    auto dx = a.pose.position.x - b.pose.position.x;
    auto dy = a.pose.position.y - b.pose.position.y;

    return hypot(dx, dy);
}

double AgentUtils::normaliseAngle(double angle)
{
    while(angle > M_PI) 
        angle -= 2.0 * M_PI;

    while(angle <= -M_PI) 
        angle += 2.0 * M_PI;
    
    return angle;
}

double AgentUtils::yawFromPose(const geometry_msgs::msg::Pose &pose)
{
    const auto quat = pose.orientation;
    return quatToYaw(quat);
}

std::string AgentUtils::getCurrentOrbitHazardPlugin()
{
    auto& config = loadConfig();
    return config.value("current_orbit_hazard_plugin", "nav2"); // default to nav2
}

OrbitHazardConfigParams AgentUtils::getOrbitHazardParams()
{
    std::map<std::string, std::variant<float, std::string, bool>> orbitHazardParamsMap;
    std::variant<float, std::string, bool> value;
    auto& config = loadConfig();

    OrbitHazardConfigParams configParams;
    
    configParams.cameraOffset = config["camera_offset"].get<float>();
    configParams.cameraClear = config["camera_clear"].get<float>();
    configParams.radialGain = config["k_radial"].get<float>();
    configParams.maxAllowedAngularVelRadsPerSec = config["max_w"].get<float>();
    configParams.cmdRateHz = config["cmd_rate"].get<float>();
    configParams.approachSpeed = config["approach_speed"].get<float>();
    configParams.headingProportionalGain = config["k_head_p"].get<float>();
    configParams.approachTolerance = config["approach_tol"].get<float>();
    configParams.targetOmegaRadsPerSec = config["target_omega_radians_per_sec"].get<float>();
    configParams.maxLinearVelMetresPerSec = config["v_cap_metres_per_sec"].get<float>();
    configParams.slowRatio = config["slow_ratio"].get<float>();
    configParams.minSpeedScalingFactor = config["min_speed_scaling_factor"].get<float>();
    configParams.angularTolerance = config["angular_tol"].get<float>();

    
    configParams.isOrbitClockwise = config["is_orbit_clockwise"].get<bool>();
    configParams.hazardOrbitSafetyMargin = config["hazard_orbit_safety_margin"].get<float>();
    configParams.hazardNavigateEntryMargin = config["hazard_navigate_entry_margin"].get<float>();
    configParams.hazardNavigatePoseMaxDistance = config["hazard_navigate_pose_max_distance"].get<float>();
    configParams.hazardMinimalPathOffset = config["hazard_minimal_path_offset"].get<float>();
    configParams.hazardCaptureIntervalDegree = config["hazard_capture_interval_degree"].get<float>();
    configParams.tfTimeoutSeconds = config["tf_timeout_sec"].get<float>();
    configParams.logEveryNTimerTicks = config["log_every_n_timer_ticks"].get<float>();
    configParams.agilexBaseFrame = config["agilex_base_frame"].get<std::string>();
    configParams.mapFrame = config["map_frame"].get<std::string>();
    configParams.nav2OrbitControllerId = config["nav2_orbit_controllerId"].get<std::string>();
    configParams.nav2GoalCheckerId = config["nav2_goal_checker_id"].get<std::string>();
    configParams.nav2ProgressCheckerId = config["nav2_progress_checker_id"].get<std::string>();
    configParams.nav2ControllerServerSetParamsSrv = config["nav2_controller_server_set_params_srv"].get<std::string>();
    configParams.nav2FollowPathAction = config["nav2_follow_path_action"].get<std::string>();
    configParams.hazardOrbitAngleTopic = config["hazard_orbit_angle_topic"].get<std::string>();
    configParams.hazardOrbitDoneTopic = config["hazard_orbit_done_topic"].get<std::string>();
    configParams.captureHazardOrbitImageTopic = config["capture_hazard_orbit_image_topic"].get<std::string>();
    configParams.nav2PluginOrbitHazardRosbagDir = config["nav2_plugin_orbit_hazard_rosbag_dir"].get<std::string>();
    configParams.twistPluginOrbitHazardRosbagDir = config["twist_plugin_orbit_hazard_rosbag_dir"].get<std::string>();

    return configParams;
}

DustSensorRelayParams AgentUtils::getDustSensorRelayParams()
{
    DustSensorRelayParams configParams;
    auto& config = loadConfig();
    configParams.MaxPublishCount = config["max_publishes_dust_sensor_relay"].get<int>();
    configParams.PublishThreshold = config["dust_sensor_relay_threshold"].get<float>();
    configParams.MinValuesAvailibity = config["dust_sensor_relay_min_values_availibity"].get<int>();

    return configParams;
}

void AgentUtils::clearFolder(std::filesystem::path folderPath)
{
    if(!std::filesystem::exists(folderPath) 
            || std::filesystem::is_empty(folderPath)) {
            return; // folder either does not exist or is empty so nothinfg to clear
    }

    for(const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        std::filesystem::remove_all(entry.path());
    }
}

std::tuple<bp::child, bp::ipstream> AgentUtils::startRosbag(const std::string& outDir)
{
    auto& config = loadConfig();
    if(!(config.contains("ros_bag_topics") && config.contains("rosbag_qos_file"))) {
        throw std::runtime_error("No configuration set for ros bag topics and/or qos file");
    }

    auto topics = config["ros_bag_topics"].get<std::vector<std::string>>();
    auto qosFile = config["rosbag_qos_file"].get<std::string>();

    // create dirs if does not exist
    std::filesystem::create_directories(std::filesystem::path(outDir).parent_path());
    std::vector<std::string> args {
        "bag",
        "record",
        "-o", outDir,
        "--qos-profile-overrides-path", qosFile
    };
    args.insert(args.end(), topics.begin(), topics.end());

    bp::ipstream err;

    bp::child proc(bp::search_path("ros2"),
                   bp::args(args),
                   bp::std_out > bp::null,
                   bp::std_err > err);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    if(!proc.running()) {
        std::string line;
        std::getline(err, line);
        throw std::runtime_error("ros2 bag start process failed- " + line);
    }

    return {std::move(proc), std::move(err)};
}

void AgentUtils::stopRosbag(boost::process::child &bagProc, bp::ipstream &err)
{
    if(!bagProc.running()) {
        std::string line;
        std::getline(err, line);

        std::cerr << "rosbag crashed: " << line << std::endl;
    } else {
        ::kill(bagProc.id(), SIGINT);

        bagProc.wait();

        std::cout << "Bag closed - loop done." << std::endl;
    }

}

float AgentUtils::getStandOffRadius(const HazardType hazardType)
{
    auto& config = loadConfig();
    float safetyRadius, safetyMargin;
    if(hazardType == HazardType::FIBROUS_HAZARD) {
        safetyRadius = config.value("fibrous_hazard_safety_radius", 0.0f);
        safetyMargin = config.value("fibrous_hazard_safety_margin", 0.0f);
    } else if(hazardType == HazardType::DUST_PLUME) {
        safetyRadius = config.value("dust_plume_safety_radius", 0.0f);
        safetyMargin = config.value("dust_plume_safety_margin", 0.0f);
    }
    else
        throw std::invalid_argument("Unknown hazard type");
    return safetyRadius + safetyMargin;

}

void AgentUtils::loadHazardPoses()
{
    if(!hazardPoses_.empty())   // already loaded, so return
        return;   

    auto& config = loadConfig();
    if(!config.contains("hazards")) {
        throw std::runtime_error("No hazards defined");
    }
    
    auto hazards = config["hazards"].get<std::vector<std::string>>();
    for(const auto& hazard : hazards) {
        if(!config.contains(hazard)) 
            continue;

        auto pose = config[hazard].get<SimplePose>();
        hazardPoses_.insert(std::make_pair(parseHazardType(hazard), pose));
    }

}

nlohmann::json AgentUtils::getConfig()
{
    nlohmann::json config;
    try {
        std::ifstream file(CONFIG_PATH);
        file >> config;
    } catch(std::runtime_error& ex) {
        std::cerr << "Error: Unable to load config";
    }
    return config;
}

nlohmann::json &AgentUtils::loadConfig()
{
    // static references are not recreated if already initialised so will get only once
    static nlohmann::json config = getConfig();
    return config;
}
