#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "orbit_hazard_plugin/orbit_hazard_manager_plugin.hpp"


class TwistCommanderOrbitHazardPlugin : public OrbitHazardManagerPlugin {
public:
    TwistCommanderOrbitHazardPlugin();

    virtual void doHazardOrbit(std::shared_ptr<OrbitHazard::Request> request) override;

    ~TwistCommanderOrbitHazardPlugin() override = default;    

private:
    
    // variables
    float accumulator_{0.0f};
    rclcpp::TimerBase::SharedPtr orbitTimer_;
    float baseLinearVelocity_;
    float slowZone_;
    float standOffRadius_;

    // methods
    void orbitLoop();

    void calculateAndSetOrbitingParameters();

    void doOrbit(const float& bearing, const float& yawError, const float& currentRadius);
    
    virtual void startOrbit() override;
};