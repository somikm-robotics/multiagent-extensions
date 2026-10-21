#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

class FakeNavigateThroughPosesServer: public rclcpp::Node 
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;

    FakeNavigateThroughPosesServer();

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
        std::shared_ptr<const NavigateThroughPoses::Goal>); 
   
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    rclcpp_action::Server<NavigateThroughPoses>::SharedPtr server_;
};