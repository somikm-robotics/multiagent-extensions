#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <memory>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "path_planner_plugin/path_planner_plugin.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

using PoseStamped = geometry_msgs::msg::PoseStamped;

class Nav2PathPlannerPlugin : public PathPlannerPlugin {
public:
    
        Nav2PathPlannerPlugin() = default;

        void initialise(const rclcpp::Node::SharedPtr & node) override;
        
        void plan_path(RequestPathPlan::Goal request,
            SendGoalOptions options) override;

        ~Nav2PathPlannerPlugin() override = default;

private:
    //Nav2 client
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr computePathToPoseClient_;

    // future handlers
    void onComputePathGoalResponse(GoalHandle::SharedPtr goal_handle);

    void computePathResult(const GoalHandle::WrappedResult& result);

    // methods
    geometry_msgs::msg::PoseStamped getPose(geometry_msgs::msg::Pose goalPose);    
    
    static std::vector<PoseStamped> decimate(nav_msgs::msg::Path path, rclcpp::Node::SharedPtr node);

    // variables
    inline static const float SPACING = 2.0;
    inline static const float EUCLID = 1e-3;
    inline static const float FIRSTPOSEDIFFERENCE = 0.05;
};