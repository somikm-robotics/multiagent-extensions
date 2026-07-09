#include "orbit_controller_plugin/nav2_orbit_controller_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "agent_utils.hpp"


namespace orbit_controller {

    bool Nav2OrbitControllerPlugin::safetyCheck(const geometry_msgs::msg::PoseStamped &pose, double v_cmd) const
    {
        if(!useCostmapSafety_ || !costmap_)
            return true;
        
        const double xr = pose.pose.position.x;
        const double yr = pose.pose.position.y;
        const double yaw = AgentUtils::yawFromPose(pose.pose);

        // Check current cell
        const unsigned char c0 = costAtWorld(xr, yr);
        if (c0 >= static_cast<unsigned char>(safetyCostThreshold_)) {
            return false;
        }

        // Check a point in front of robot (in its heading direction)
        // This catches when you are about to drive into inflated obstacle, etc.
        const double look = std::max(0.0, safetyLookAhead_);
        const double xf = xr + std::cos(yaw) * look;
        const double yf = yr + std::sin(yaw) * look;

        const unsigned char cf = costAtWorld(xf, yf);
        if (cf >= static_cast<unsigned char>(safetyCostThreshold_)) {
            (void)v_cmd; // reserved for future: scale lookahead by speed
            return false;
        }

        return true;

    }
    unsigned char Nav2OrbitControllerPlugin::costAtWorld(double wx, double wy) const
    {
        if(!costmap_)
            return 0;
        
        unsigned int mx, my;
        if(!costmap_->worldToMap(wx, wy, mx, my)) 
            return 255; // outside map: treat as unsafe
        
        return costmap_->getCost(mx, my);
    }

    void Nav2OrbitControllerPlugin::resetOrbitState()
    {
        firstIteration_ = true;
        orbitComplete_ = false;
        prevAngle_ = 0.0;
        accumulatedAngle_ = 0.0;
        blockedHold_ = 0;

        publishDonePub();
    }

    void Nav2OrbitControllerPlugin::publishBlockedPub(bool blockedVal)
    {
        if(blockedPub_) {
            std_msgs::msg::Bool blocked;
            blocked.data = blockedVal;
            blockedPub_->publish(blocked);
        }
    }

    void Nav2OrbitControllerPlugin::publishDonePub()
    {
        if(donePub_) {
            std_msgs::msg::Bool doneMsg;
            doneMsg.data = orbitComplete_;
            donePub_->publish(doneMsg);
        }
    }

    void Nav2OrbitControllerPlugin::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                              std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                                              const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmapRos_ = costmapRos;

        if(costmapRos_) {
            costmap_ = costmapRos_->getCostmap();
        }

        // Parameters (declared under controller plugin name, e.g. Orbit.*)
  node_->declare_parameter(name_ + ".center_x", 0.0);
  node_->declare_parameter(name_ + ".center_y", 0.0);
  node_->declare_parameter(name_ + ".radius", 0.0);
  node_->declare_parameter(name_ + ".linear_vel", 0.2);
  node_->declare_parameter(name_ + ".gain", 1.5);

  node_->declare_parameter(name_ + ".isClockwiseDirection", true); // "clockwise" - true or "counter_clockwise" - false
  node_->declare_parameter(name_ + ".slow_down_start_deg", 330.0);

  node_->declare_parameter(name_ + ".use_costmap_safety", true);
  node_->declare_parameter(name_ + ".safety_lookahead", 0.35);
  node_->declare_parameter(name_ + ".safety_cost_threshold", 253);
  node_->declare_parameter(name_ + ".safety_stop_ticks", 8);

  // Get initial values
  node_->get_parameter(name_ + ".center_x", centerX_);
  node_->get_parameter(name_ + ".center_y", centerY_);
  node_->get_parameter(name_ + ".radius", radius_);
  node_->get_parameter(name_ + ".linear_vel", linearVel_);
  node_->get_parameter(name_ + ".gain", gain_);
  node_->get_parameter(name_ + ".slow_down_start_deg", slowdownStartDeg_);

  node_->get_parameter(name_ + ".use_costmap_safety", useCostmapSafety_);
  node_->get_parameter(name_ + ".safety_lookahead", safetyLookAhead_);
  node_->get_parameter(name_ + ".safety_cost_threshold", safetyCostThreshold_);
  node_->get_parameter(name_ + ".safety_stop_ticks", safetyStopTicks_);

  bool isClockwise;
  node_->get_parameter(name_ + ".isClockwiseDirection", isClockwise);
  if (!isClockwise) {
    directionSign_ = +1;
  } else {
    directionSign_ = -1; // default clockwise
  }

  // Publishers
  anglePub_ = node_->create_publisher<std_msgs::msg::Float32>("/orbit_angle", 10);
  donePub_ = node_->create_publisher<std_msgs::msg::Bool>("/orbit_done", 10);
  blockedPub_ = node_->create_publisher<std_msgs::msg::Bool>("/orbit_blocked", 10);

  paramCbHandle_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
        for(const auto& p : params) {
            const auto& name = p.get_name();

            // Center/radius updates
            if(name == name_ + ".center_x") {
                centerX_ = p.as_double();
                needReset_.store(true);
            } 
            else if(name == name_ + ".center_y") { 
                centerY_ = p.as_double();
                needReset_.store(true);
            } 
            else if(name == name_ + ".radius") {
                radius_ = std::max(0.05, p.as_double());
                needReset_.store(true);
            }
            else if (name == name_ + ".direction") {
                const auto s = p.as_string();
                directionSign_ = (s == "counter_clockwise" || s == "ccw") ? +1 : -1;
                needReset_.store(true);
            }

            // Safety
            else if (name == name_ + ".use_costmap_safety") 
                useCostmapSafety_ = p.as_bool();
            else if (name == name_ + ".safety_lookahead") 
                safetyLookAhead_ = std::max(0.0, p.as_double());
            else if (name == name_ + ".safety_cost_threshold") 
                safetyCostThreshold_ = p.as_int();
            else if (name == name_ + ".safety_stop_ticks") 
                safetyStopTicks_ = std::max(1, static_cast<int>(p.as_int()));

            // Stop / slow-down behavior
            else if (name == name_ + ".slow_down_start_deg") 
                slowdownStartDeg_ = p.as_double();
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason = "";

        return res;
    }
  );
  resetOrbitState();
}

    void Nav2OrbitControllerPlugin::cleanup()
    {
        anglePub_.reset();
        donePub_.reset();
        blockedPub_.reset();
        paramCbHandle_.reset();
        costmap_ = nullptr;
        costmapRos_.reset();
        tf_.reset();
        node_.reset();
    }

    void Nav2OrbitControllerPlugin::activate()
    {
        resetOrbitState();
    }

    void Nav2OrbitControllerPlugin::deactivate()
    {
        // intentionally empty
    }

    geometry_msgs::msg::TwistStamped Nav2OrbitControllerPlugin::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, 
        const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *)
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header = pose.header;

        if(needReset_.exchange(false)) {
            resetOrbitState(); // safe here
        }

        // If previously blocked, hold stop for a few ticks to avoid chatter
        // or If already complete, keep stopped
        if(blockedHold_ > 0 || orbitComplete_) {
            if(blockedHold_ > 0)
                blockedHold_--;
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;

            publishBlockedPub(true);

            return cmd;
        }

        const double xr = pose.pose.position.x;
        const double yr = pose.pose.position.y;
        
        const double dx = xr - centerX_;
        const double dy = yr - centerY_;
        const double d = std::sqrt(dx * dx + dy * dy);

        // Angle around center
        const double currentAngle_ = std::atan2(dy, dx);

        if(firstIteration_) {
            prevAngle_ = currentAngle_;
            firstIteration_ = false;
        }

        const double delta = AgentUtils::normaliseAngle(currentAngle_ - prevAngle_);
        accumulatedAngle_ += std::abs(delta);
        prevAngle_ = currentAngle_;
        
        if(anglePub_) {
            std_msgs::msg::Float32 angle;
            angle.data = static_cast<float>(accumulatedAngle_ * 180.0 / M_PI);
            anglePub_->publish(angle);
        }
        
        // Orbit completion check (>= 360 degrees)
        if(accumulatedAngle_ >= 2.0 * M_PI) {
            orbitComplete_ = true;

            publishDonePub();

            // Start smooth stop: command near-zero and return (manager can switch controllers after it sees orbit_done)
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;

            publishBlockedPub(false);

            return cmd;
        }
        // Base orbit control
        const double radialError = d - radius_;

        // Tangential angular velocity magnitude v/r, sign sets clockwise vs counter-clockwise
        const double omegaBase = static_cast<double>(directionSign_) * (linearVel_ / std::max(0.05, radius_));

        // Radial correction: sign must match direction to stabilize (same sign as omega_base)
        // (For clockwise: direction_sign_=-1 => omega becomes more negative when too far, curving inward)
        const double omega = omegaBase + static_cast<double>(directionSign_) * (gain_ * radialError);

        // Smooth slow-down near end of orbit
        const double deg = accumulatedAngle_ * 180.0 / M_PI;
        double vCmd = linearVel_;
        double wCmd = omega;

        if(deg >= slowdownStartDeg_) {
            const double span = std::max(1.0, 360.0 - slowdownStartDeg_);
            const double t = std::clamp((360.0 - deg) / span, 0.0, 1.0);  // 1 -> 0 as we approach 360
            vCmd = (0.2 + 0.8 *t); // never drop below 20% until completion threshold
            wCmd = (0.2 + 0.8 *t); 
        }

        // Costmap safety check (stop if unsafe)
        if (!safetyCheck(pose, vCmd)) {
            blockedHold_ = safetyStopTicks_;
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;

            publishBlockedPub(true);
            return cmd;
        }

        // command outout
        cmd.twist.linear.x = vCmd;
        cmd.twist.angular.z = wCmd;

        publishBlockedPub(false);

        return cmd;
    }

    void Nav2OrbitControllerPlugin::setPlan(const nav_msgs::msg::Path &)
    {
        // Orbit controller ignores global plan intentionally
    }

    void Nav2OrbitControllerPlugin::setSpeedLimit(const double &speedLimit, const bool &percentage)
    {
        if(percentage) {
            // percentage of current linear velocity
            linearVel_ *= speedLimit;
        } else {
            // absolute speed limit
            linearVel_ = std::min(linearVel_, speedLimit);
        }
    }
} // namespace orbit_controller
PLUGINLIB_EXPORT_CLASS(orbit_controller::Nav2OrbitControllerPlugin, nav2_core::Controller);