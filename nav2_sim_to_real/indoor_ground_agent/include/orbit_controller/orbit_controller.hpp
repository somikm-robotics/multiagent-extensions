#ifndef ORBIT_CONTROLLER__ORBIT_CONTROLLER_HPP_
#define ORBIT_CONTROLLER__ORBIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"


namespace orbit_controller
{

class OrbitController : public nav2_core::Controller
{
public:
  OrbitController() = default;
  ~OrbitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

   void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // --- helpers ---
  static double normalizeAngle(double a);
  static double yawFromPose(const geometry_msgs::msg::Pose & p);

  bool safetyCheck(const geometry_msgs::msg::PoseStamped & pose, double v_cmd) const;
  unsigned char costAtWorld(double wx, double wy) const;

  void resetOrbitState();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::atomic_bool need_reset_{false};


  // TF not used directly here but kept for interface completeness
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blocked_pub_;

  // Dynamic parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Orbit parameters (set by Orbiting_Manager)
  double center_x_{0.0};
  double center_y_{0.0};
  double radius_{0.0};
  double linear_vel_{0.2};
  double gain_{1.5};

  // direction: -1 = clockwise, +1 = counter-clockwise
  int direction_sign_{-1};

  // Stop behavior
  double slow_down_start_deg_{330.0};   // begin slowing after this many degrees
  double stop_linear_thresh_{0.02};     // m/s, below this we stop
  double stop_angular_thresh_{0.05};    // rad/s, below this we stop

  // Costmap safety parameters
  bool use_costmap_safety_{true};
  double safety_lookahead_{0.35};       // meters ahead to check
  int safety_cost_threshold_{253};      // 253+ usually lethal/inscribed-ish; tune per your costmap
  int safety_stop_ticks_{8};            // how many cycles to hold stop once blocked

  // Internal state for 1 revolution
  bool first_iteration_{true};
  bool orbit_complete_{false};

  double prev_angle_{0.0};
  double accumulated_angle_{0.0};  // radians

  // Additional “blocked” hysteresis
  mutable int blocked_hold_{0};
};

}  // namespace orbit_controller

#endif  // ORBIT_CONTROLLER__ORBIT_CONTROLLER_HPP_