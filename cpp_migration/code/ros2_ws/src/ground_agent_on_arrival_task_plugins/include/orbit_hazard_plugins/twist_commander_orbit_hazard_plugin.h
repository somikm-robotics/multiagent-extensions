#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class TwistCommanderOrbitHazardPlugin : public OrbitHazardPlugin {

private:
    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twistPub;    

    // Subscriptions
  
    void loop();
    void doRadialIn(const float& radialError, const float& yawError);
    void doRadialOut(const float& radialError, const float& yawError);
    void do_tangentAlign(const float& bearing, const float& yawError);
    void do_orbit(const float& bearing, const float& yawError, const float& currentRadius);

    static float wrap(const float& angle);
    static float quatToYaw(const geometry_msgs::msg::Quaternion& quat);


public:
    TwistCommanderOrbitHazardPlugin()=default;

    void do_hazard_orbit(OrbitHazard::Request request,
        OrbitHazard::Response response) override;

    ~TwistCommanderOrbitHazardPlugin() = default;
};