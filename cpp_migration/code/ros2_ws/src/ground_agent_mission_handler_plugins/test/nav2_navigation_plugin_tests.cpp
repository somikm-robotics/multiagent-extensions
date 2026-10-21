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
#include "test_helpers/nav2/fake_nav2_navigate_through_poses_server.hpp"
#include "test_helpers/nav2/fake_nav2_spin_server.hpp"


// Fix to use Test_F as part of cleaning
class Nav2NavigationPluginTests : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override {
        testNode_ = std::make_shared<rclcpp::Node>("test_node");
        RCLCPP_INFO(testNode_->get_logger(), "Testing - in setup");
        //methodName_ = GetParam();

        executor_.add_node(testNode_);

        if(GetParam() == "handleNavigation") {
            auto fakeNavigateThroughPosesServer = std::make_shared<FakeNavigateThroughPosesServer>();
            fakeNodes_.push_back(fakeNavigateThroughPosesServer);
            executor_.add_node(fakeNavigateThroughPosesServer);
        } else if (GetParam() == "completeRotation"){
            auto fakeSpinServer = std::make_shared<FakeSpinServer>();
            fakeNodes_.push_back(fakeSpinServer);
            executor_.add_node(fakeSpinServer);
        }

        spinThread_= std::thread([this]() {
            executor_.spin();
        });
        
        testPluginLoader_ = std::make_unique<pluginlib::ClassLoader<NavigationPluginBase>>(
                            "ground_agent_mission_handler_plugins", 
                            "NavigationPluginBase");
        
        RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin loader created");                                    
        testPlugin_ = testPluginLoader_->createSharedInstance("Nav2NavigationPlugin");
        
        RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin created");
    }

    void TearDown() override {
        executor_.cancel();
        if(spinThread_.joinable()) {
            spinThread_.join();
        }

        for(auto& fakeNode: fakeNodes_)  {
            executor_.remove_node(fakeNode);
            fakeNode.reset();
        }
        executor_.remove_node(testNode_);
        testNode_.reset();
    }

    rclcpp::Node::SharedPtr testNode_;
    std::vector<rclcpp::Node::SharedPtr> fakeNodes_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinThread_;
    std::unique_ptr<pluginlib::ClassLoader<NavigationPluginBase>> testPluginLoader_;
    std::shared_ptr<NavigationPluginBase> testPlugin_;
    rclcpp::Subscription<transportMsgs::NavigationResult>::SharedPtr navResultSubscription_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr rotationCompletedSubscription_;
    std::string methodName_;

};

TEST_P(Nav2NavigationPluginTests, HandleNavigationTest)
{
    RCLCPP_INFO(testNode_->get_logger(), "Testing - started");
    std::atomic_bool messageReceived {false};

    navResultSubscription_ = testNode_->create_subscription<transportMsgs::NavigationResult>("reached_target", 10,
        [&](const transport_interfaces::msg::NavigationResult result){
            if(result.success)
                RCLCPP_INFO(testNode_->get_logger(), "Navigation completed successfully");
            messageReceived = true;
        });

    testPlugin_->initialise(testNode_);
    RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin initialised");
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

    testPlugin_->handleNavigation(request);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(messageReceived);        
}

TEST_P(Nav2NavigationPluginTests, CompleteRotationTest)
{
    RCLCPP_INFO(testNode_->get_logger(), "Testing - started");
    std::atomic_bool messageReceived {false};

    rotationCompletedSubscription_ = testNode_->create_subscription<transportMsgs::MissionCompleted>("completed_rotation", 10,
        [&](const transport_interfaces::msg::MissionCompleted result){
            if(result.success)
                RCLCPP_INFO(testNode_->get_logger(), "Rotation completed successfully");
            messageReceived = true;
        });

    testPlugin_->initialise(testNode_);
    RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin initialised");

    auto request = std::make_shared<RotateToPosition::Request>();
    request->current_yaw = 20.0;
    request->target_yaw = 0.0;

    testPlugin_->completeRotation(request);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(messageReceived);        
}



INSTANTIATE_TEST_SUITE_P(
    Nav2NavigationPlugin, Nav2NavigationPluginTests,
    ::testing::Values(
        "handleNavigation", "completeRotation")
);

int main(int argc, char** argv) {

    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}
