#include "orbit_hazard_plugin/twist_commander_orbit_hazard_plugin.hpp"
#include "agent_utils.hpp"

TwistCommanderOrbitHazardPlugin::TwistCommanderOrbitHazardPlugin() 
{
	accum_ = 0;
}

void TwistCommanderOrbitHazardPlugin::doHazardOrbit(std::shared_ptr<OrbitHazard::Request> request)
{
	setFromRequestParams(request);
    navigateToStagingPose(configParams_.nav2PluginOrbitHazardRosbagDir);
    if(stagingCompleted_) {
		calculateAndSetOrbitingParameters();
		orbitTimer_ = node_->create_wall_timer(std::chrono::duration<double>(1/configParams_.cmdRateHz),
                                             std::bind(&TwistCommanderOrbitHazardPlugin::orbitLoop, this));  
        this->startOrbit();
	}
}
void TwistCommanderOrbitHazardPlugin::startOrbit()
{
	// intentionally blank
}

void TwistCommanderOrbitHazardPlugin::calculateAndSetOrbitingParameters()
{
	standOffRadius_ = AgentUtils::calculateOrbitSafetyRadius(hazardType_, estimatedHazardDiameter_);
	slowZone_ = configParams_.slowRatio * standOffRadius_;
	baseLinearVelocity_ = std::min(configParams_.maxLinearVelMetresPerSec, 
									configParams_.targetOmegaRadsPerSec * standOffRadius_);
}

void TwistCommanderOrbitHazardPlugin::orbitLoop()
{
	if(!currentYaw_.has_value() || !inProgress_) 
        return;

	tick_++;
	auto [ radius, bearing] = calculateRadiusAndBearing();
	auto desiredYaw = bearing - M_PI/2;
	auto yawError = AgentUtils::normaliseAngle(desiredYaw - currentYaw_.value());

	this->doOrbit(bearing, yawError, radius);
}

void TwistCommanderOrbitHazardPlugin::doOrbit(const float &bearing, const float &yawError, const float &currentRadius)
{
	if(previousBearing_.has_value()) {
		auto dtheta = AgentUtils::normaliseAngle(bearing - previousBearing_.value());
		accum_+= -dtheta;
	}
	previousBearing_ = bearing;

	if(accum_ >= 2 * M_PI) {
		geometry_msgs::msg::Twist twist;
		twistPub_->publish(twist);    // stop once
		RCLCPP_INFO(node_->get_logger(), "Orbit complete - exiting.");
		AgentUtils::stopRosbag(bagProc_, bagErr_);
		if(!orbitTimer_->is_canceled())
			orbitTimer_->cancel();
		publishOrbitCompleted(true);
		shutdownGracefully();
	}

	// FEED‑FORWARD + RADIAL‑P CONTROLLER  (CW orbit, ROS +z is CCW)
	auto radialError = currentRadius - radius_;  // + if we’re outside, – if inside

	 //1. SCALE SPEED  (slow if |radialError| > SLOW_ZONE)
	 auto scale = std::max(configParams_.minSpeedScalingFactor, 1 - abs(radialError/slowZone_));
	 auto linearVel = baseLinearVelocity_ * scale;

	//2. FEED‑FORWARD + RADIAL‑P   (compute *curvature*, then ω)
	auto feedforwardCurvature = 1.0f/radius_;  // ideal curvature
	auto pCorrectedCurvature = configParams_.radialGain * radialError / radius_;   // P correction

	// keep full steering authority even when creeping
	auto angularVel = (feedforwardCurvature + pCorrectedCurvature) * baseLinearVelocity_;
	if(configParams_.isOrbitClockwise) {
		angularVel = - angularVel;  // CW negative → minus
	} 

	// clamps to safe range: -maxAllowedAngularVelRadsPerSec <= angularVel <= maxAllowedAngularVelRadsPerSec
	angularVel = std::max(std::min(angularVel, configParams_.maxAllowedAngularVelRadsPerSec), 
									-configParams_.maxAllowedAngularVelRadsPerSec);  // saturate safely

	geometry_msgs::msg::Twist twist;
	twist.linear.x = linearVel;
	twist.angular.z = angularVel;	
	twistPub_->publish(twist);		
	
	if(tick_ % configParams_.logEveryNTimerTicks == 0) {
        RCLCPP_INFO_STREAM(node_->get_logger(), getLogMessage(yawError, twist, radialError));
    }
}


PLUGINLIB_EXPORT_CLASS(TwistCommanderOrbitHazardPlugin, OrbitHazardManagerPluginBase)