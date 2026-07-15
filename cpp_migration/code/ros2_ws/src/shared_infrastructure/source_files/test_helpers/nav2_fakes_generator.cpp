#include "test_helpers/nav2/fake_nav2_follow_path_server.hpp"
#include "test_helpers/nav2/fake_nav2_set_parameter_server.hpp"
#include "test_helpers/nav2/fake_nav2_follow_waypoints_server.hpp"
#include "test_helpers/nav2/fake_nav2_compute_path_to_pose_server.hpp"

FakeNav2FollowPathServer::FakeNav2FollowPathServer() : Node("fake_follow_path_server")
{
    donePub_ = this->create_publisher<std_msgs::msg::Bool>("/orbit_done", 10);
    server_ = rclcpp_action::create_server<FollowPath>(
        this, "/follow_path",
        std::bind(&FakeNav2FollowPathServer::handleGoal, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(&FakeNav2FollowPathServer::handleCancel, this,
            std::placeholders::_1),
        std::bind(&FakeNav2FollowPathServer::handleAccepted, this,
            std::placeholders::_1));
}
rclcpp_action::GoalResponse FakeNav2FollowPathServer::handleGoal(
                                            const rclcpp_action::GoalUUID &, 
                                            std::shared_ptr<const FollowPath::Goal>)
{
    RCLCPP_INFO(this->get_logger(), "In handle goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeNav2FollowPathServer::handleCancel(
                                        const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "In handle cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeNav2FollowPathServer::handleAccepted(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "In handle accepted");

    //  RCLCPP_INFO(this->get_logger(), "handle accepted - creating result");

    auto result = std::make_shared<FollowPath::Result>();
    
    //result->missed_waypoints = {};

    // RCLCPP_INFO(this->get_logger(), "handle accepted - created result");
    
    // RCLCPP_INFO(this->get_logger(), "handle accepted - set params");
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "handle accepted - done");
    std_msgs::msg::Bool msg;
    msg.data = true;
    donePub_ ->publish(msg);
}

FakeSetParameterServer::FakeSetParameterServer() : Node("fake_nav2_set_parameter_server") 
{
    fakeService_ = this->create_service<SetParameters>("/controller_server/set_parameters",
                                                    std::bind(&FakeSetParameterServer::handleRequest,
                                                                this, std::placeholders::_1, std::placeholders::_2));
}

void FakeSetParameterServer::handleRequest(std::shared_ptr<SetParameters::Request> request, std::shared_ptr<SetParameters::Response> response)
{
    response->results.resize(request->parameters.size());

    for(auto& result : response->results) {
        result.successful =  true;
        result.reason = "";
    }
}

FakeFollowWaypointsServer::FakeFollowWaypointsServer() : Node("fake_follow_waypoints_server")
{
    server_ = rclcpp_action::create_server<FollowWaypoints>(
            this, "/follow_waypoints",
            std::bind(&FakeFollowWaypointsServer::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeFollowWaypointsServer::handleCancel, this,
                std::placeholders::_1),
            std::bind(&FakeFollowWaypointsServer::handleAccepted, this,
                std::placeholders::_1));
}
rclcpp_action::GoalResponse FakeFollowWaypointsServer::handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowWaypoints::Goal>)
{
    RCLCPP_INFO(this->get_logger(), "In handle goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeFollowWaypointsServer::handleCancel(const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "In handle cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeFollowWaypointsServer::handleAccepted(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "In handle accepted");

    //  RCLCPP_INFO(this->get_logger(), "handle accepted - creating result");

    auto result = std::make_shared<FollowWaypoints::Result>();
    
    result->missed_waypoints = {};

    // RCLCPP_INFO(this->get_logger(), "handle accepted - created result");
    
    result->missed_waypoints = {};
    // RCLCPP_INFO(this->get_logger(), "handle accepted - set params");
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "handle accepted - done");
}

FakeComputePathToPoseServer::FakeComputePathToPoseServer() : Node("fake_cmpute_path_server")
{
    server_ = rclcpp_action::create_server<ComputePathToPose>(
            this, "compute_path_to_pose",
            std::bind(&FakeComputePathToPoseServer::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeComputePathToPoseServer::handleCancel, this,
                std::placeholders::_1),
            std::bind(&FakeComputePathToPoseServer::handleAccepted, this,
                std::placeholders::_1));
}
rclcpp_action::GoalResponse FakeComputePathToPoseServer::handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePathToPose::Goal>)
{
    RCLCPP_INFO(this->get_logger(), "In handle goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeComputePathToPoseServer::handleCancel(const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "In handle cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeComputePathToPoseServer::handleAccepted(const std::shared_ptr<GoalHandle> goalHandle)
{
    RCLCPP_INFO(this->get_logger(), "In handle accepted");

    RCLCPP_INFO(this->get_logger(), "handle accepted - creating result");

    auto result = std::make_shared<ComputePathToPose::Result>();
    
    RCLCPP_INFO(this->get_logger(), "handle accepted - created result");
    
    //result->path =
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->get_clock()->now();

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = 1.0;
    pose1.pose.position.y = 2.0;
    pose1.pose.orientation.w = 1.0;
    waypoints.push_back(pose1);

    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 3.0;
    pose2.pose.position.y = 4.0;
    pose2.pose.orientation.w = 1.0;
    waypoints.push_back(pose2);

    geometry_msgs::msg::PoseStamped pose3;
    pose3.header.frame_id = "map";
    pose3.pose.position.x = 5.0;
    pose3.pose.position.y = 6.0;
    pose3.pose.orientation.w = 1.0;
    waypoints.push_back(pose3);
    path.poses = waypoints;
    result->path = path;

    RCLCPP_INFO(this->get_logger(), "handle accepted - set params");
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "handle accepted - done");
}
