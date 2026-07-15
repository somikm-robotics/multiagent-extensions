#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>

class FakeComputePathToPoseServer: public rclcpp::Node 
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    FakeComputePathToPoseServer();

    private:
    rclcpp_action::Server<ComputePathToPose>::SharedPtr server_;
    
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ComputePathToPose::Goal>);

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);

    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);
};