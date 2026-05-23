#include "orbit_controller/orbit_controller.hpp"

#include <cmath>
#include <algorithm>

#include <pluginlib/class_list_macros.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace orbit_controller
{

// --- Utility ---
double OrbitController::normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double OrbitController::yawFromPose(const geometry_msgs::msg::Pose & p)
{
  // yaw from quaternion (x,y,z,w)
  const auto & q = p.orientation;
  // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// --- Lifecycle ---
void OrbitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  if (costmap_ros_) {
    costmap_ = costmap_ros_->getCostmap();
  }

  // Parameters (declared under controller plugin name, e.g. Orbit.*)
  node_->declare_parameter(name_ + ".center_x", 0.0);
  node_->declare_parameter(name_ + ".center_y", 0.0);
  node_->declare_parameter(name_ + ".radius", 0.0);
  node_->declare_parameter(name_ + ".linear_vel", 0.2);
  node_->declare_parameter(name_ + ".gain", 1.5);

  node_->declare_parameter(name_ + ".direction", std::string("clockwise")); // "clockwise" or "counter_clockwise"
  node_->declare_parameter(name_ + ".slow_down_start_deg", 330.0);

  node_->declare_parameter(name_ + ".use_costmap_safety", true);
  node_->declare_parameter(name_ + ".safety_lookahead", 0.35);
  node_->declare_parameter(name_ + ".safety_cost_threshold", 253);
  node_->declare_parameter(name_ + ".safety_stop_ticks", 8);

  // Get initial values
  node_->get_parameter(name_ + ".center_x", center_x_);
  node_->get_parameter(name_ + ".center_y", center_y_);
  node_->get_parameter(name_ + ".radius", radius_);
  node_->get_parameter(name_ + ".linear_vel", linear_vel_);
  node_->get_parameter(name_ + ".gain", gain_);
  node_->get_parameter(name_ + ".slow_down_start_deg", slow_down_start_deg_);

  node_->get_parameter(name_ + ".use_costmap_safety", use_costmap_safety_);
  node_->get_parameter(name_ + ".safety_lookahead", safety_lookahead_);
  node_->get_parameter(name_ + ".safety_cost_threshold", safety_cost_threshold_);
  node_->get_parameter(name_ + ".safety_stop_ticks", safety_stop_ticks_);

  std::string dir;
  node_->get_parameter(name_ + ".direction", dir);
  if (dir == "counter_clockwise" || dir == "ccw") {
    direction_sign_ = +1;
  } else {
    direction_sign_ = -1; // default clockwise
  }

  // Publishers
  angle_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/orbit_angle", 10);
  done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/orbit_done", 10);
  blocked_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/orbit_blocked", 10);

  // Dynamic parameter updates (so Orbiting_Manager can change center/radius/direction at runtime)
  param_cb_handle_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    {
      for (const auto & p : params)
      {
        const auto & n = p.get_name();

        // Center/radius updates
        if (n == name_ + ".center_x") { 
            center_x_ = p.as_double(); 
            need_reset_.store(true); 
        }
        else if (n == name_ + ".center_y") { 
            center_y_ = p.as_double(); 
            need_reset_.store(true); }
        else if (n == name_ + ".radius")   
        { 
            radius_ = std::max(0.05, p.as_double()); 
            need_reset_.store(true); 
        }
        else if (n == name_ + ".direction") {
            const auto s = p.as_string();
            direction_sign_ = (s == "counter_clockwise" || s == "ccw") ? +1 : -1;
            need_reset_.store(true);
        }

        // Safety
        else if (n == name_ + ".use_costmap_safety") use_costmap_safety_ = p.as_bool();
        else if (n == name_ + ".safety_lookahead") safety_lookahead_ = std::max(0.0, p.as_double());
        else if (n == name_ + ".safety_cost_threshold") safety_cost_threshold_ = p.as_int();
        else if (n == name_ + ".safety_stop_ticks") safety_stop_ticks_ = std::max(1, static_cast<int>(p.as_int()));

        // Stop / slow-down behavior
        else if (n == name_ + ".slow_down_start_deg") slow_down_start_deg_ = p.as_double();
      }

      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      r.reason = "";
      return r;
    });

  resetOrbitState();
}

void OrbitController::cleanup()
{
  angle_pub_.reset();
  done_pub_.reset();
  blocked_pub_.reset();
  param_cb_handle_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void OrbitController::activate()
{
  resetOrbitState();
}

void OrbitController::deactivate()
{
  // nothing special
}

void OrbitController::setPlan(const nav_msgs::msg::Path &)
{
  // Orbit controller ignores global plan intentionally
}

// --- Orbit state ---
void OrbitController::resetOrbitState()
{
  first_iteration_ = true;
  orbit_complete_ = false;
  prev_angle_ = 0.0;
  accumulated_angle_ = 0.0;
  blocked_hold_ = 0;

  // publish done false (optional, but helps manager)
  if (done_pub_) {
    std_msgs::msg::Bool d; d.data = false;
    done_pub_->publish(d);
  }
}

// --- Costmap helpers ---
unsigned char OrbitController::costAtWorld(double wx, double wy) const
{
  if (!costmap_) return 0;

  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    // outside map: treat as unsafe
    return 255;
  }
  return costmap_->getCost(mx, my);
}

bool OrbitController::safetyCheck(const geometry_msgs::msg::PoseStamped & pose, double v_cmd) const
{
  if (!use_costmap_safety_) return true;
  if (!costmap_) return true;

  const double xr = pose.pose.position.x;
  const double yr = pose.pose.position.y;
  const double yaw = yawFromPose(pose.pose);

  // Check current cell
  const unsigned char c0 = costAtWorld(xr, yr);
  if (c0 >= static_cast<unsigned char>(safety_cost_threshold_)) {
    return false;
  }

  // Check a point in front of robot (in its heading direction)
  // This catches when you are about to drive into inflated obstacle, etc.
  const double look = std::max(0.0, safety_lookahead_);
  const double xf = xr + std::cos(yaw) * look;
  const double yf = yr + std::sin(yaw) * look;

  const unsigned char cf = costAtWorld(xf, yf);
  if (cf >= static_cast<unsigned char>(safety_cost_threshold_)) {
    (void)v_cmd; // reserved for future: scale lookahead by speed
    return false;
  }

  return true;
}

void OrbitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    // percentage of current linear velocity
    linear_vel_ = linear_vel_ * speed_limit;
  } else {
    // absolute speed limit
    linear_vel_ = std::min(linear_vel_, speed_limit);
  }
}

// --- Main control ---
geometry_msgs::msg::TwistStamped OrbitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  if (need_reset_.exchange(false)) {
    resetOrbitState();  // safe here
    }

  // If previously blocked, hold stop for a few ticks to avoid chatter
  if (blocked_hold_ > 0) {
    blocked_hold_--;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    if (blocked_pub_) { 
        std_msgs::msg::Bool b; 
        b.data = true; 
        blocked_pub_->publish(b); 
    }
    return cmd;
  }

  // If already complete, keep stopped
  if (orbit_complete_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    if (blocked_pub_) { 
        std_msgs::msg::Bool b; 
        b.data = false; 
        blocked_pub_->publish(b); 
    }
    return cmd;
  }

  const double xr = pose.pose.position.x;
  const double yr = pose.pose.position.y;

  const double dx = xr - center_x_;
  const double dy = yr - center_y_;
  const double d = std::sqrt(dx * dx + dy * dy);

  // Angle around center
  const double current_angle = std::atan2(dy, dx);

  if (first_iteration_) {
    prev_angle_ = current_angle;
    first_iteration_ = false;
  }

  const double delta = normalizeAngle(current_angle - prev_angle_);
  accumulated_angle_ += std::abs(delta);
  prev_angle_ = current_angle;

  // Publish accumulated angle in degrees
  if (angle_pub_) {
    std_msgs::msg::Float32 a;
    a.data = static_cast<float>(accumulated_angle_ * 180.0 / M_PI);
    angle_pub_->publish(a);
  }

  // Orbit completion check (>= 360 degrees)
  if (accumulated_angle_ >= 2.0 * M_PI) {
    orbit_complete_ = true;

    if (done_pub_) {
      std_msgs::msg::Bool dmsg;
      dmsg.data = true;
      done_pub_->publish(dmsg);
    }

    // Start smooth stop: command near-zero and return (manager can switch controllers after it sees orbit_done)
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    if (blocked_pub_) { std_msgs::msg::Bool b; b.data = false; blocked_pub_->publish(b); }
    return cmd;
  }

  // Base orbit control
  const double radial_error = d - radius_;

  // Tangential angular velocity magnitude v/r, sign sets clockwise vs counter-clockwise
  const double omega_base = static_cast<double>(direction_sign_) * (linear_vel_ / std::max(0.05, radius_));

  // Radial correction: sign must match direction to stabilize (same sign as omega_base)
  // (For clockwise: direction_sign_=-1 => omega becomes more negative when too far, curving inward)
  const double omega = omega_base + static_cast<double>(direction_sign_) * (gain_ * radial_error);

  // Smooth slow-down near end of orbit
  const double deg = accumulated_angle_ * 180.0 / M_PI;
  double v_cmd = linear_vel_;
  double w_cmd = omega;

  if (deg >= slow_down_start_deg_) {
    const double span = std::max(1.0, 360.0 - slow_down_start_deg_);
    const double t = std::clamp((360.0 - deg) / span, 0.0, 1.0);  // 1 -> 0 as we approach 360
    // Slow down aggressively but smoothly
    v_cmd *= (0.2 + 0.8 * t);  // never drop below 20% until completion threshold
    w_cmd *= (0.2 + 0.8 * t);
  }

  // Costmap safety check (stop if unsafe)
  if (!safetyCheck(pose, v_cmd)) {
    blocked_hold_ = safety_stop_ticks_;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    if (blocked_pub_) { 
        std_msgs::msg::Bool b; 
        b.data = true; 
        blocked_pub_->publish(b); 
    }
    return cmd;
  }

  // Command output
  cmd.twist.linear.x = v_cmd;
  cmd.twist.angular.z = w_cmd;

  if (blocked_pub_) { 
    std_msgs::msg::Bool b; 
    b.data = false; 
    blocked_pub_->publish(b); 
}
  return cmd;
}

}  // namespace orbit_controller

PLUGINLIB_EXPORT_CLASS(orbit_controller::OrbitController, nav2_core::Controller)