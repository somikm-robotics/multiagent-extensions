#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "pluginlib/class_loader.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "path_planner_plugin/nav2_path_planner_plugin.hpp"
#include "transport_interfaces/action/request_path_plan.hpp"


using RequestPathPlan = transport_interfaces::action::RequestPathPlan;

class FakeComputePathToPoseServer: public rclcpp::Node 
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    FakeComputePathToPoseServer() : Node("fake_cmpute_path_server") {
        server_ = rclcpp_action::create_server<ComputePathToPose>(
            this, "compute_path_to_pose",
            std::bind(&FakeComputePathToPoseServer::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeComputePathToPoseServer::handleCancel, this,
                std::placeholders::_1),
            std::bind(&FakeComputePathToPoseServer::handleAccepted, this,
                std::placeholders::_1));
    }
private:
    rclcpp_action::Server<ComputePathToPose>::SharedPtr server_;
//    rclcpp::TimerBase::SharedPtr resultTimer_;
    
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ComputePathToPose::Goal>) 
    {
        RCLCPP_INFO(this->get_logger(), "In handle goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "In handle cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle)
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
};
using GoalHandleRequestPathPlan = rclcpp_action::ClientGoalHandle<RequestPathPlan>;

TEST(Nav2PathPlannerPluginTests, PathPlanningSuccess)
{
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    RCLCPP_INFO(testNode->get_logger(), "Nav2 Path Planner plug in test - started test");
     
    auto fakeAction = std::make_shared<FakeComputePathToPoseServer>();

    std::atomic_bool computePathSuccess{false};

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(testNode);
    executor.add_node(fakeAction);

    std::thread spinThread(
        [&]() {
            executor.spin();
        });
    
    RCLCPP_INFO(testNode->get_logger(), "Nav2 Path Planner plug in test - loading plugin");
    pluginlib::ClassLoader<PathPlannerPluginBase> testPluginLoader(
        "ground_agent_mission_handler_plugins", 
        "PathPlannerPluginBase"    
    );
    RCLCPP_INFO(testNode->get_logger(), "Nav2 Path Planner plug in test - created plugin loader");
    auto testPlugin = testPluginLoader.createSharedInstance("Nav2PathPlannerPlugin");
    RCLCPP_INFO(testNode->get_logger(), "Nav2 Path Planner plug in test - created plugin");
   
    testPlugin->initialise(testNode);
    RCLCPP_INFO(testNode->get_logger(), "Nav2 Path Planner plug in test - initialised plugin");
    RequestPathPlan::Goal request;
    geometry_msgs::msg::Pose destinationPose;
    destinationPose.position.x = 1.0;
    destinationPose.position.y = 2.0;
    destinationPose.orientation.w = 1.0;
    
    request.goal = destinationPose;
    SendGoalOptions sendOptions;
    
    sendOptions.result_callback = [testNode, &computePathSuccess] (const GoalHandleRequestPathPlan::WrappedResult &result) {
        int size = result.result->waypoints.size();
        RCLCPP_INFO_STREAM(testNode->get_logger(), "Received waypoints: " << size);
        if(size > 1 && result.result->success) {
            RCLCPP_INFO_STREAM(testNode->get_logger(), "Message received: " << result.result->message);
            computePathSuccess = true;
        }
            
    };

    sendOptions.goal_response_callback = [testNode] (GoalHandleRequestPathPlan::SharedPtr goalHandle) {
        if(!goalHandle) {
            RCLCPP_ERROR(testNode->get_logger(), "Path Plan failed");
        }
        RCLCPP_INFO(testNode->get_logger(), "Path Plan failed");
    };
    sendOptions.feedback_callback = [testNode] (GoalHandleRequestPathPlan::SharedPtr goalHaldle, 
                                    const std::shared_ptr<const RequestPathPlan::Feedback> feedback) {
        RCLCPP_INFO(testNode->get_logger(), "Feedback: %s", feedback->status.c_str());
    };

    RCLCPP_INFO(testNode->get_logger(), "Nav Path Planner plug in test - requested plan path");
    testPlugin->plan_path(request, sendOptions);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !computePathSuccess &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(computePathSuccess);
    executor.cancel();
    spinThread.join();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    testing::InitGoogleTest(&argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}