#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"


class Nav2PathPlannerPlugin : public PathPlannerPlugin {


private:
    //Nav2 client
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr _computePathToPoseClient;

    //client callback
    void onPathReturn(rclcpp::Client<PathPlanResult>::SharedFuture future);

    // future handlers
    onComputePathGoalResponse(
        rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalFuture future);

    // methods
    geometry_msgs::msg::PoseStamped getPose(geometry_msgs::msg::Pose goalPose);    
    void sendPathPlanResult(std::vector<geometry_msgs::msg::PoseStamped>& waypoints);

public:
    
        Nav2PathPlannerPlugin() = default;

        void plan_path(RequestPathPlan::Request request,
            RequestPathPlan::Response response) override;

        ~Nav2PathPlannerPlugin = default;
};