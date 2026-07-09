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

namespace orbit_controller {

    class Nav2OrbitControllerPlugin: public nav2_core::Controller {
        private:
            // publishers    
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr anglePub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr donePub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blockedPub_;
        
            // variables
            rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
            std::string name_;
            std::atomic_bool needReset_{false};
        
            // TF not used directly here but kept for interface completeness
            std::shared_ptr<tf2_ros::Buffer> tf_;
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos_;
            nav2_costmap_2d::Costmap2D* costmap_{nullptr};

            // Dynamic parameter callback handle
            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;

            // Orbit parameters (set by Orbiting_Manager)
            double centerX_{0.0};
            double centerY_{0.0};
            double radius_{0.0};
            double linearVel_{0.2};
            double gain_{1.5};

            // direction: -1 = clockwise, +1 = counter-clockwise
            int directionSign_{-1};

            // Stop behavior
            double slowdownStartDeg_{330.0};   // begin slowing after this many degrees
            double stopLinearThresh_{0.02};     // m/s, below this we stop
            double stopAngularThresh_{0.05};    // rad/s, below this we stop

            // Costmap safety parameters
            bool useCostmapSafety_{true};
            double safetyLookAhead_{0.35};       // meters ahead to check
            int safetyCostThreshold_{253};      // 253+ usually lethal/inscribed-ish; tune per your costmap
            int safetyStopTicks_{8};            // how many cycles to hold stop once blocked

            // Internal state for 1 revolution
            bool firstIteration_{true};
            bool orbitComplete_{false};

            double prevAngle_{0.0};
            double accumulatedAngle_{0.0};  // radians

            // Additional “blocked” hysteresis
            mutable int blockedHold_{0};

            // methods
            bool safetyCheck(const geometry_msgs::msg::PoseStamped & pose, double v_cmd) const;
            unsigned char costAtWorld(double wx, double wy) const;
            void resetOrbitState();
            void publishBlockedPub(bool blockedVal);
            void publishDonePub();

        public:
            Nav2OrbitControllerPlugin() = default;
            ~Nav2OrbitControllerPlugin() override = default;

            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name,
                const std::shared_ptr<tf2_ros::Buffer> tf,
                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos) override;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            geometry_msgs::msg::TwistStamped computeVelocityCommands(
                const geometry_msgs::msg::PoseStamped & pose,
                const geometry_msgs::msg::Twist & velocity,
                nav2_core::GoalChecker * goal_checker) override;

            void setPlan(const nav_msgs::msg::Path & path) override;

            void setSpeedLimit(const double & speed_limit, const bool & percentage) override;


    };
} // namespace orbit_controller