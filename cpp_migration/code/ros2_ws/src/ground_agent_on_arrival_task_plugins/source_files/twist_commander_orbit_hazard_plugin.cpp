#include "twist_commander_orbit_hazard_plugin.h"

void TwistCommanderOrbitHazardPlugin::do_hazard_orbit(OrbitHazard::Request request, OrbitHazard::Response response)
{
}

void TwistCommanderOrbitHazardPlugin::loop()
{
}

void TwistCommanderOrbitHazardPlugin::doRadialIn(const float& radialError, const float& yawError)
{
}

void TwistCommanderOrbitHazardPlugin::doRadialOut(const float& radialError, const float& yawError)
{
}

void TwistCommanderOrbitHazardPlugin::do_tangentAlign(const float& bearing, const float& yawError)
{
}

void TwistCommanderOrbitHazardPlugin::do_orbit(const float& bearing, const float& yawError, const float& currentRadius)
{
}

float TwistCommanderOrbitHazardPlugin::wrap(const float& angle)
{
	return 0.0f;
}

float TwistCommanderOrbitHazardPlugin::quatToYaw(const geometry_msgs::msg::Quaternion& quat)
{
	return 0.0f;
}
