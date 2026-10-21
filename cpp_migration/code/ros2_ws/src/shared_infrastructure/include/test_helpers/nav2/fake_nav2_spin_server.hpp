#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/spin.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

class FakeSpinServer: public rclcpp::Node 
{
public:
    using Spin = nav2_msgs::action::Spin;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Spin>;

    FakeSpinServer();

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
        std::shared_ptr<const Spin::Goal>); 
   
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    rclcpp_action::Server<Spin>::SharedPtr server_;
};