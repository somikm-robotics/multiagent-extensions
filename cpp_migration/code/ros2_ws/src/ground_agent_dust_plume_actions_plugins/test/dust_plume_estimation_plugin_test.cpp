#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <filesystem>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <boost/process.hpp>


#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "dust_plume_estimation_plugin/dust_plume_estimation_plugin.hpp"

#include "test_helpers/fake_image_publisher_node.hpp"

using namespace std::chrono_literals;
namespace bp = boost::process;

class DustPlumeEstimationPluginTest : public testing::Test {
protected:
    void SetUp() override {
        testNode_ = std::make_shared<rclcpp::Node>("test_node");

        RCLCPP_INFO(testNode_->get_logger(), "Dust Plume Estimation Plugin test started");
        densityRegressorServiceProcess_ = std::make_unique<bp::child>(
            bp::search_path("ros2"),
            bp::args({
                "run",
                "ai_python",
                "density_regressor_node"
            }),
            bp::std_out > stdout,
            bp::std_err > stderr);
        
        auto densityRegressorClient = testNode_->create_client<DensityRegressor>("density_regressor");

        ASSERT_TRUE(densityRegressorClient->wait_for_service(10s));
        RCLCPP_INFO(testNode_->get_logger(), "Density Regressor Service Process started");

        pluginLoader_ = std::make_unique<pluginlib::ClassLoader<DustPlumeEstimationPluginBase>>(
            "ground_agent_dust_plume_actions_plugins",
            "DustPlumeEstimationPluginBase"
        );
        plugin_ = pluginLoader_->createSharedInstance("DustPlumeEstimationPlugin");

        RCLCPP_INFO(testNode_->get_logger(), "Dust Plume Estimation Plugin loaded");
    }

    void TearDown() override {
        if(densityRegressorServiceProcess_ && densityRegressorServiceProcess_->running()) {
            densityRegressorServiceProcess_->terminate();
            densityRegressorServiceProcess_->wait();
        }
    }

    rclcpp::Node::SharedPtr testNode_;
    std::unique_ptr<bp::child> densityRegressorServiceProcess_;
    std::unique_ptr<pluginlib::ClassLoader<DustPlumeEstimationPluginBase>> pluginLoader_;
    std::shared_ptr<DustPlumeEstimationPluginBase> plugin_;
};
TEST_F(DustPlumeEstimationPluginTest, PublishReceivedMessage) {

    std::atomic_bool messageReceived{false};
    auto fakeImagePubNode = std::make_shared<FakeImagePublisherNode>();

    RCLCPP_INFO(testNode_->get_logger(), "Created image publisher");
    auto estimatedDensitySub = testNode_->create_subscription<transportMsgs::DensityEstimationResult>(
                                "estimation_complete", 10,
                                [&](const transportMsgs::DensityEstimationResult result ){
                                    RCLCPP_INFO_STREAM(testNode_->get_logger(), "Density: " << result.density
                                                        << " Severity Level: " << result.level);
                                    messageReceived = true;
                                });
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(testNode_);
    executor.add_node(fakeImagePubNode);
    RCLCPP_INFO(testNode_->get_logger(), "added nodes to executor");                                

    std::thread spinThread(
        [&]() {
            executor.spin();
        });   

   // RCLCPP_INFO(testNode_->get_logger(), "started spin");
    plugin_->initialise(testNode_);
   // RCLCPP_INFO(testNode_->get_logger(), "initialised plugin");
    auto request = std::make_shared<PlumeDensity::Request>();
    plugin_->estimatePlumeDensity(request);

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