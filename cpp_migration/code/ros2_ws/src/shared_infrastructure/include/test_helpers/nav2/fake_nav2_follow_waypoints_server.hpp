#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

class FakeFollowWaypointsServer: public rclcpp::Node 
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

    FakeFollowWaypointsServer(); 

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowWaypoints::Goal>); 
   
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    rclcpp_action::Server<FollowWaypoints>::SharedPtr server_;
};
