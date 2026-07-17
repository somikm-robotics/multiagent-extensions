#include "orbit_hazard_plugin/twist_commander_orbit_hazard_plugin.hpp"
#include "agent_utils.hpp"

TwistCommanderOrbitHazardPlugin::TwistCommanderOrbitHazardPlugin() :
	OrbitHazardManagerPlugin("TwistCommanderOrbitHazardPlugin") 
{
}

void TwistCommanderOrbitHazardPlugin::doHazardOrbit(std::shared_ptr<OrbitHazard::Request> request)
{
	setFromRequestParams(request);
    navigateToStagingPose(configParams_.nav2PluginOrbitHazardRosbagDir);
    
	if(stagingCompleted_) {
		orbitMode_ = OrbitMode::ORBIT;
		this->startOrbit();
	}
		
}

void TwistCommanderOrbitHazardPlugin::startOrbit()
{
	if(stagingCompleted_) {
		orbitMode_ = OrbitMode::ORBIT;
		
		startCamera();
		calculateAndSetOrbitingParameters();

		if(auto node = node_.lock()) {
			orbitTimer_ = node->create_wall_timer(std::chrono::duration<double>(1/configParams_.cmdRateHz),
                                             std::bind(&TwistCommanderOrbitHazardPlugin::orbitLoop, this));  
		} else {
        	auto msg = "Some exception occurred - Node not available";
			RCLCPP_ERROR(logger_, msg);
			return;
		}
	}     
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
//	RCLCPP_INFO(logger_, "In Orbit loop");
	if(!currentYaw_.has_value() || !inProgress_) 
        return;

	//RCLCPP_INFO(logger_, "In Orbit loop 2");
	tick_++;
	auto [ radius, bearing] = calculateRadiusAndBearing();
	auto desiredYaw = bearing - M_PI/2;
	auto yawError = AgentUtils::normaliseAngle(desiredYaw - currentYaw_.value());

	this->doOrbit(bearing, yawError, radius);
}

void TwistCommanderOrbitHazardPlugin::doOrbit(const float &bearing, const float &yawError, const float &currentRadius)
{
	RCLCPP_INFO(logger_, "In do Orbit - %s", AgentUtils::toOrbitModeStr(orbitMode_).c_str());
	if(previousBearing_.has_value()) {
		auto dtheta = AgentUtils::normaliseAngle(bearing - previousBearing_.value());
		accumulator_+= -dtheta;
	}
	previousBearing_ = bearing;

	if(accumulator_ >= 2 * M_PI) {
		geometry_msgs::msg::Twist twist;
		twistPub_->publish(twist);    // stop once
		RCLCPP_INFO(logger_, "Orbit complete - exiting.");
		AgentUtils::stopRosbag(bagProc_, bagErr_);
		if(!orbitTimer_->is_canceled())
			orbitTimer_->cancel();
		publishOrbitCompleted(true);
		cleanup();
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
		//RCLCPP_INFO(logger_, "Orbit mode in orbit loop: %s", AgentUtils::toOrbitModeStr(orbitMode_).c_str());
        RCLCPP_INFO_STREAM(logger_, getLogMessage(yawError, twist, radialError));
    }
}


PLUGINLIB_EXPORT_CLASS(TwistCommanderOrbitHazardPlugin, OrbitHazardManagerPluginBase)