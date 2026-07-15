#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "pluginlib/class_loader.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin.hpp"
#include "test_helpers/fake_dust_sensor_replay_publisher_node.hpp"

TEST(DustSensorRelayPluginTests, PublishSuccess)
{
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto fakePub = std::make_shared<FakeDustSensorRelayPublisherNode>();
    std::atomic_bool messageReceived{false};
    int maxCount = 3;
    std::atomic_int ctr{0};
    auto resultSub = testNode->create_subscription<transport_interfaces::msg::DustSensorResult>("dust_sensor_reading", 10,
        [&](const transport_interfaces::msg::DustSensorResult result){
            ++ctr;
            RCLCPP_INFO_STREAM(testNode->get_logger(), "Result received: Times - " << ctr);
            if(ctr >= maxCount) {
                messageReceived = true;
                RCLCPP_INFO_STREAM(testNode->get_logger(), "Result received: Total Times" << ctr);
            }
        });
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(testNode);
    executor.add_node(fakePub);

    std::thread spinThread(
        [&]() {
            executor.spin();
        });   

    pluginlib::ClassLoader<DustSensorRelayPluginBase> testPluginLoader(
        "ground_agent_dust_plume_actions_plugins",
        "DustSensorRelayPluginBase");

    auto testPlugin = testPluginLoader.createSharedInstance("DustSensorRelayPlugin");
    testPlugin->initialise(testNode);

   
    testPlugin->publishDustSensorResult(); 
 
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
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