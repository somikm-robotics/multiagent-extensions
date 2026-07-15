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
#include "test_helpers/nav2/fake_nav2_compute_path_to_pose_server.hpp"

using RequestPathPlan = transport_interfaces::action::RequestPathPlan;
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