#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include "pluginlib/class_loader.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "navigation_plugin/nav2_navigation_plugin.hpp"
#include "transport_interfaces/msg/navigation_result.hpp"
#include "agent_utils.hpp"
#include "test_helpers/nav2/fake_nav2_follow_waypoints_server.hpp"

TEST(Nav2NavigationPluginTests, PublishSuccessMessage)
{
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto fakeAction = std::make_shared<FakeFollowWaypointsServer>();
    std::atomic_bool messageReceived{false};
    auto resultSub = testNode->create_subscription<transport_interfaces::msg::NavigationResult>("reached_target", 10,
        [&](const transport_interfaces::msg::NavigationResult result){
            messageReceived = true;
        });
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(testNode);
    executor.add_node(fakeAction);

    std::thread spinThread(
        [&]() {
            executor.spin();
        });


    pluginlib::ClassLoader<NavigationPluginBase> testPluginLoader(
        "ground_agent_mission_handler_plugins", 
        "NavigationPluginBase");
    auto testPlugin = testPluginLoader.createSharedInstance("Nav2NavigationPlugin");
    
    testPlugin->initialise(testNode);
    auto request = std::make_shared<Navigate::Request>();
    request->hazard_type = AgentUtils::toHazardTypeString(HazardType::DUST_PLUME);
    
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

    request->waypoints = waypoints;
    testPlugin->handleNavigation(request);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(messageReceived);
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