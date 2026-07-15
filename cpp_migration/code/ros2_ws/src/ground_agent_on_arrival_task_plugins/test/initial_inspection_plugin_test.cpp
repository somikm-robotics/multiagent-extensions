#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <filesystem>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "initial_inspection_plugin/initial_inspection_plugin.hpp"
#include "agent_utils.hpp"
#include "shared_infrastructure/enums.hpp"

#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"
#include "test_helpers/fake_image_publisher_node.hpp"

std::string getFirstFolderName(std::filesystem::path imagePath) {
    auto itr = std::filesystem::directory_iterator(imagePath);

    if(itr != std::filesystem::directory_iterator() && itr->is_directory()) {
        return itr->path().filename().string();
    }
    return "";
}

TEST(InitialInspectionPluginTests, SaveSuccess) {
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto fakeImagePub = std::make_shared<FakeImagePublisherNode>();
    auto fakeOdomPub = std::make_shared<FakeOdomPublisherNode>();
    auto fakeTwistSub = std::make_shared<FakeTwistSubscriberNode>();

    std::atomic_bool savedImages {false};
    auto imagesFolderPath = AgentUtils::getImageOutputFolderPath();
    auto imagePath = std::filesystem::path(imagesFolderPath) / "initial_inspection";

    AgentUtils::clearFolder(imagePath);

    auto responseSub = testNode->create_subscription<transportMsgs::MissionCompleted>(
                                    "inspection_complete", 10, [&](const transportMsgs::MissionCompleted msg){
                                        if(msg.success && std::filesystem::exists(imagePath) &&
                                            !std::filesystem::is_empty(imagePath)) {
                                                
                                                auto folderName = getFirstFolderName(imagePath);
                                                auto imageCapturePath = imagePath / folderName;
                                                RCLCPP_INFO_STREAM(testNode->get_logger(), "image captured path " << imageCapturePath);
                                                int count = std::count_if(std::filesystem::directory_iterator(imageCapturePath),
                                                                            std::filesystem::directory_iterator(),
                                                                        [](const auto& entry){
                                                                            return entry.is_regular_file();
                                                                        });
                                                
                                                 RCLCPP_INFO(testNode->get_logger(), "Count: %d", count);
                                                if(count == 10)
                                                    savedImages = true;
                                            }
                                    });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(testNode);
    executor.add_node(fakeImagePub);
    executor.add_node(fakeOdomPub);
    executor.add_node(fakeTwistSub);

    std::thread spinThread(
        [&]() {
            executor.spin();
        });   

    pluginlib::ClassLoader<InitialInspectionPluginBase> testPluginLoader(
        "ground_agent_on_arrival_task_plugins",
        "InitialInspectionPluginBase");
    
    auto testPlugin = testPluginLoader.createSharedInstance("InitialInspectionPlugin");
    
    testPlugin->initialise(testNode);

    auto request = std::make_shared<InitialInspection::Request>();
    request->hazard_type = AgentUtils::toHazardTypeString(HazardType::FIBROUS_HAZARD);
    auto response = testPlugin->performInspection(request);
    ASSERT_TRUE(!response->message.empty());

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !savedImages &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(savedImages);
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