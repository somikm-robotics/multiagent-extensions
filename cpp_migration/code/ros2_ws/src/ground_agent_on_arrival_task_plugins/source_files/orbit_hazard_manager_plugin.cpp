#include "orbit_hazard_plugin/orbit_hazard_manager_plugin.hpp"
#include "agent_utils.hpp"
#include <iomanip>
#include <algorithm>

void OrbitHazardManagerPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
    node_ = node;

    // Publishers
    orbitCompletedPub_ = node->create_publisher<transportMsgs::MissionCompleted>(
        "orbit_completed", 10);
    twistPub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // subscription
    odomSub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](const nav_msgs::msg::Odometry& msg){
        this->odomCallback(msg);
    });

    stagingX_ = 0.0;
    stagingY_ = 0.0;
    stagingYaw_ = 0.0;

    configParams_ = AgentUtils::getOrbitHazardParams();
	previousBearing_ = std::nullopt;
	orbitMode_ = OrbitMode::NONE;
	tick_ = 0;

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, node_);

    stagingCompleted_ = false;
    inProgress_ = false;
    orbitCompleted_ = false;
    serviceTimeout_ = AgentUtils::getServiceTimeoutFromConfig();

}

void OrbitHazardManagerPlugin::restart()
{
    if(inProgress_ || orbitCompleted_)
        return;
    inProgress_ = true;
    if(stagingCompleted_)
        if(!stagingTimer_->is_canceled()) {
            stagingTimer_->cancel();
        }
        startOrbit();
}

void OrbitHazardManagerPlugin::odomCallback(const nav_msgs::msg::Odometry& msg)
{
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header = msg.header;
    pose.pose = msg.pose.pose;

    auto transformStampedOpt = lookupTransform();
    if(!transformStampedOpt.has_value())
        return;
    
    auto transformStamped = transformStampedOpt.value();
    currentX_ = transformStamped.transform.translation.x;
    currentY_ = transformStamped.transform.translation.y;
    currentYaw_ = AgentUtils::quatToYaw(transformStamped.transform.rotation);

}

void OrbitHazardManagerPlugin::navigateToStagingPose(std::string rosBagDir)
{
    if(stagingCompleted_) {
        if(!stagingTimer_->is_canceled()) {
            stagingTimer_->cancel();
        }
        return;
    }
    
    bagDir_ = rosBagDir;
    orbitMode_ = OrbitMode::RADIAL_IN; // default
    RCLCPP_INFO(node_->get_logger(), "Trying to Navigate to orbit staging position...");
    inProgress_ = true;
    
    tfIssuesTimer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&OrbitHazardManagerPlugin::restart, this));  
    stagingTimer_ = node_->create_wall_timer(std::chrono::duration<double>(1/configParams_.cmdRateHz),
                                             std::bind(&OrbitHazardManagerPlugin::stagingLoop, this));    
}

std::tuple<float, float> OrbitHazardManagerPlugin::calculateRadiusAndBearing()
{
        // geometry
    auto dx = currentX_- hazardCenterPointX_;
    auto dy = currentY_ - hazardCenterPointY_;
    auto radius = hypot(dx, dy);
    auto bearing = atan2(dx, dy);                    //CCW from +X

    return std::tuple<float, float>(radius, bearing);
}

void OrbitHazardManagerPlugin::setFromRequestParams(std::shared_ptr<OrbitHazard::Request> request)
{
    hazardCenterPointX_ = request->hazard_center_point.x;
    hazardCenterPointY_ = request->hazard_center_point.y;
    estimatedHazardDiameter_ = request->estimated_hazard_diameter;
    hazardType_ = AgentUtils::parseHazardType(request->hazard_type);
    auto patchHalfDiagonal = hypot(estimatedHazardDiameter_/2, estimatedHazardDiameter_/2);

    radius_ = patchHalfDiagonal + configParams_.cameraClear + configParams_.cameraOffset;
}

void OrbitHazardManagerPlugin::stagingLoop()
{
    if(!currentYaw_.has_value() || !inProgress_) 
        return;

    if(stagingCompleted_ && !stagingTimer_->is_canceled())
        stagingTimer_->cancel();

    tick_++;

    // only intially
    if(tick_ == 1) {
        auto initialRadius = hypot(currentX_ - hazardCenterPointX_, 
                                    currentY_ - hazardCenterPointY_);
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << initialRadius;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Initial Radial distance: " << ss.str());
    }

    auto [radius, bearing] = calculateRadiusAndBearing();
    
    float desiredYaw;
    if(orbitMode_ == OrbitMode::RADIAL_IN) {
        desiredYaw = AgentUtils::normaliseAngle(bearing + M_PI); // R → C  (inwards) - towards centre
    } else if(orbitMode_ == OrbitMode::RADIAL_OUT) {
        desiredYaw = AgentUtils::normaliseAngle(bearing); // away from centre
    }
    
    auto yawError = AgentUtils::normaliseAngle(desiredYaw - currentYaw_.value());
    if(tick_ < 10) {
        RCLCPP_INFO_STREAM(node_->get_logger(), 
        "RAW pos = ("
        << std::fixed << std::setprecision(2)
        << std::showpos << currentX_ << ","
        << currentY_ << ") "
        << "rad = " << radius
        << " err = " << (radius - radius_));
    }

    // sign flip because CW is positive for us
    auto radialError = radius - radius_;  // # + outside, – inside
     //If we were told to go RADIAL_IN but we are already inside,
     //flip to RADIAL_OUT automatically (and vice‑versa).
    if(orbitMode_ == OrbitMode::RADIAL_IN && radialError < 0)
        orbitMode_ = OrbitMode::RADIAL_OUT; 
    else if(orbitMode_ == OrbitMode::RADIAL_OUT and radialError > 0) {
        orbitMode_ = OrbitMode::RADIAL_IN;
    }

    switch (orbitMode_)
    {
        case OrbitMode::RADIAL_IN:
            return this->doRadialIn(radialError, yawError);
            break;
        case OrbitMode::RADIAL_OUT:
            return this->doRadialOut(radialError, yawError);
            break;
        case OrbitMode::TANGENT_ALIGN:
            return this->doTangentAlign(bearing, yawError);
            break;
        default:
            return;
    }
}

void OrbitHazardManagerPlugin::doRadialIn(const float &radialError, const float &yawError)
{
    if(abs(radialError) < configParams_.approachTolerance) {
        orbitMode_ = OrbitMode::TANGENT_ALIGN;
        RCLCPP_INFO(node_->get_logger(), "RADIAL‑IN complete → TANGENT_ALIGN");
        return;
    }

    auto fwdGain = std::max(0.0f, cos(yawError)) * std::max(0.0f, radialError); 
    geometry_msgs::msg::Twist twist;
    twist.linear.x = configParams_.approachSpeed * fwdGain;
    twist.angular.z = configParams_.headingProportionalGain * yawError;
    twistPub_->publish(twist);

    if(tick_ % configParams_.logEveryNTimerTicks == 0) {
        RCLCPP_INFO_STREAM(node_->get_logger(), getLogMessage(yawError, twist, radialError));
    }
    
}

void OrbitHazardManagerPlugin::doRadialOut(const float &radialError, const float &yawError)
{
    if(abs(radialError) < configParams_.approachTolerance) {
        orbitMode_ = OrbitMode::TANGENT_ALIGN;
        RCLCPP_INFO(node_->get_logger(), "RADIAL‑OUT complete → TANGENT_ALIGN");
        return;
    }

    auto outError = -radialError;
    auto fwdGain = std::max(0.0f, cos(yawError)) * std::max(0.0f, outError); 
    geometry_msgs::msg::Twist twist;
    twist.linear.x = configParams_.approachSpeed * fwdGain;
    twist.angular.z = configParams_.headingProportionalGain * yawError;
    twistPub_->publish(twist);

    if(tick_ % configParams_.logEveryNTimerTicks == 0) {
        RCLCPP_INFO_STREAM(node_->get_logger(), getLogMessage(yawError, twist, radialError));
    }

    return;
}

void OrbitHazardManagerPlugin::doTangentAlign(const float &bearing, const float &yawError)
{
    if (abs(yawError) < (configParams_.angularTolerance * M_PI / 180.0)) {
        auto currentTimeStamp = AgentUtils::getCurrentTimeFormatString();
        auto bagDir = AgentUtils::getFolderPathStr(bagDir_, "bag_" + currentTimeStamp);
        RCLCPP_INFO_STREAM(node_->get_logger(), "TANGENT_ALIGN → Starting Ros Bag..." << bagDir);
        auto bagReturn = AgentUtils::startRosbag(bagDir);
        bagProc_ = std::move(std::get<0>(bagReturn));
        bagErr_ = std::move(std::get<1>(bagReturn));
        RCLCPP_INFO(node_->get_logger(), "TANGENT_ALIGN → Ros Bag started...");
        stagingCompleted_ = true;
        previousBearing_ = bearing;
        RCLCPP_INFO(node_->get_logger(), "TANGENT_ALIGN complete...back to the plugin for orbiting");
    }

    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.8 * copysign(1.0f, yawError); // pure spin
    twistPub_->publish(twist);

    if(tick_ % configParams_.logEveryNTimerTicks == 0) {
        RCLCPP_INFO_STREAM(node_->get_logger(), getLogMessage(yawError, twist));
    }
    return;
}

std::string OrbitHazardManagerPlugin::getLogMessage(float yawError, geometry_msgs::msg::Twist twist, std::optional<float> radialError)
{
    std::ostringstream ss;
    ss << std::left << std::setw(14) << AgentUtils::toOrbitModeStr(orbitMode_)
    << " | radial error "
    << std::showpos << std::fixed << std::setprecision(3)
    << (radialError.has_value() ? std::to_string(radialError.value()) : "N.A.")
    << " m | yaw error "
    << std::setprecision(1)
    << yawError * 180.0 / M_PI << "° | v "
    << std::setprecision(2)
    << twist.linear.x << " | ω "
    << twist.angular.z;

    return ss.str();

}

std::optional<geometry_msgs::msg::TransformStamped> OrbitHazardManagerPlugin::lookupTransform()
{
    try {
        // Ensure transform is available
        if(!tfBuffer_->canTransform(configParams_.mapFrame, configParams_.agilexBaseFrame,
                                    rclcpp::Time(), 
                                    rclcpp::Duration::from_seconds(configParams_.tfTimeoutSeconds))) 
            RCLCPP_WARN_STREAM(node_->get_logger(), "TF unavailable when computing staging pose");
            inProgress_ = false;
            return std::nullopt;

            auto tf = tfBuffer_->lookupTransform(configParams_.mapFrame, configParams_.agilexBaseFrame,
                                        rclcpp::Time(), 
                                        rclcpp::Duration::from_seconds(configParams_.tfTimeoutSeconds));
            return tf;
    } catch(const std::runtime_error& ex) {

            RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception occurred whilst looking up transform-" << ex.what());
            inProgress_ = false;
            return std::nullopt;

    }
}


void OrbitHazardManagerPlugin::populateOrientation(geometry_msgs::msg::PoseStamped& pose)
{
    tf2::Quaternion q;
        q.setRPY(0.0, 0.0, stagingYaw_);       

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
}

void OrbitHazardManagerPlugin::shutdownGracefully()
{
    if(!tfIssuesTimer_->is_canceled())
        tfIssuesTimer_->cancel();
    //rclcpp::shutdown();
}

void OrbitHazardManagerPlugin::publishOrbitCompleted(bool success)
{
    transportMsgs::MissionCompleted missionCompletedMsg;
    missionCompletedMsg.success = success;
    orbitCompletedPub_->publish(missionCompletedMsg);
    this->orbitCompleted_ = true;
}

/* std::optional<SimplePose> OrbitHazardManagerPlugin::computeStagingPose()
{
    // calculate staging istance
    auto distWithCoords = calculateStagingDistance();
    if(!distWithCoords.has_value())
        return std::nullopt;
    auto dx = distWithCoords.value().x;
    auto dy= distWithCoords.value().y;
    auto d = distWithCoords.value().distance;

    if(d < 1e-6)
        return std::nullopt;

    // calculate safe staging radius
    auto ux = dx/d;
    auto uy = dy/d;
    auto radiusEntry = estimatedHazardDiameter_ + configParams_.hazardNavigateEntryMargin;

    // Compute staging point (stay on same side)
    auto xs = hazardCenterPointX_ + radiusEntry * ux;
    auto ys = hazardCenterPointY_ + radiusEntry * uy;

    dx = xs - hazardCenterPointX_;
    dy = ys - hazardCenterPointY_;

    
    float tx, ty;
    // Compute tangential heading
    if(configParams_.isOrbitClockwise) {
        tx = -dy;
        ty = dx;
    } else {
        tx = -uy;
        ty = ux;
    }

    float yaw = atan2(ty, tx) + M_PI;
    yaw = AgentUtils::normaliseAngle(yaw);
    
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << hazardCenterPointX_ << ", " << hazardCenterPointY_;

    RCLCPP_INFO_STREAM(node_->get_logger(), "Orbit Center- (" << ss.str() << ")" );
    ss.clear();
    ss << std::fixed << std::setprecision(2) << radiusEntry;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Entry Radius: " << ss.str() );
    ss.clear();
    ss << std::fixed << std::setprecision(2) << xs << ", " << ys;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Computed Staging: " << ss.str() );

    return SimplePose {xs, ys, 0.0,  yaw};
}

std::optional<DistanceWithCoOrds> OrbitHazardManagerPlugin::calculateStagingDistance()
{
    auto tfOpt = lookupTransform();
    if(!tfOpt.has_value()) 
        return std::nullopt;

    auto tf = tfOpt.value();
    //Robot position in map frame
    float xr = tf.transform.translation.x;
    float yr = tf.transform.translation.y;

    // Compute radial direction (hazard → robot)
    auto dx = xr - hazardCenterPointX_;
    auto dy = yr - hazardCenterPointY_;
    auto d = hypot(dx, dy);

    return DistanceWithCoOrds {dx, dy, d};
}
 */
