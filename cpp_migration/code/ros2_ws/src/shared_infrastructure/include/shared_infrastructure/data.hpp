#pragma once

#include "enums.hpp"
#include <string>
#include <nlohmann/json.hpp>

struct SimplePose {
    float x;
    float y;
    float z;
    float yaw;
};


struct HazardTypePose {
    HazardType hazardType;
    SimplePose hazardPose;
    float hazardDimeter;
};

void from_json(const nlohmann::json &config, SimplePose& pose);

void from_json(const nlohmann::json &config, HazardType& hazardType);

void from_json(const nlohmann::json &config, HazardTypePose& hazardTypePose);

struct OrbitHazardConfigParams {
    bool isOrbitClockwise;
    float hazardOrbitSafetyMargin;
    float hazardNavigateEntryMargin;
    float hazardNavigatePoseMaxDistance;
    float hazardCaptureIntervalDegree;
    float tfTimeoutSeconds;
    float hazardMinimalPathOffset;
    int logEveryNTimerTicks;
    std::string agilexBaseFrame;
    std::string mapFrame;
    std::string nav2OrbitControllerId;
    std::string nav2GoalCheckerId;
    std::string nav2ProgressCheckerId;
    std::string nav2ControllerServerSetParamsSrv;
    std::string nav2FollowPathAction;
    std::string hazardOrbitAngleTopic;
    std::string hazardOrbitDoneTopic;
    std::string captureHazardOrbitImageTopic;
    std::string twistPluginOrbitHazardRosbagDir;
    std::string nav2PluginOrbitHazardRosbagDir;
    float cameraOffset;
    float cameraClear;
    float radialGain;
    float maxAllowedAngularVelRadsPerSec;
    float cmdRateHz;
    float approachSpeed;
    float headingProportionalGain;
    float approachTolerance;
    float angularTolerance;
    float targetOmegaRadsPerSec;
    float maxLinearVelMetresPerSec;
    float slowRatio;
    float minSpeedScalingFactor;
    
};

struct DistanceWithCoOrds {
    float x;
    float y;
    float distance;
};

struct DustSensorRelayParams {
    int MaxPublishCount;
    float PublishThreshold;
    int MinValuesAvailibity;
};

struct TFLookupParams {
    std::string BaseFrame;
    std::string OdomFrame;
    std::string MapFrame;
    float TFLookupTimeoutInSecs;
};
