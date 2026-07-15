#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/follow_path.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

class FakeNav2FollowPathServer: public rclcpp::Node 
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

     FakeNav2FollowPathServer();

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowPath::Goal>);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle>);

    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    rclcpp_action::Server<FollowPath>::SharedPtr server_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr donePub_;
};

