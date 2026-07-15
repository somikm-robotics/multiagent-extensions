#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"
#include "test_helpers/fake_tf_publisher_node.hpp"
#include "test_helpers/fake_image_publisher_node.hpp"
#include "test_helpers/nav2/fake_nav2_follow_path_server.hpp"
#include "test_helpers/nav2/fake_nav2_set_parameter_server.hpp"

# include "orbit_hazard_plugin/nav2_orbit_hazard_manager_plugin.hpp"
#include "agent_utils.hpp"

enum class OrbitHazardManagerPluginType {Nav2, Twist};

struct OrbitHazardManagerPluginTestData {
    std::string pluginName;
    OrbitHazardManagerPluginType pluginType;
    std::string pluginTypeName;
};

class OrbitHazardManagerPluginTests : public ::testing::TestWithParam<OrbitHazardManagerPluginTestData>
{
public:
    void SetUp() override 
    {
        testNode_ = std::make_shared<rclcpp::Node>("test_node");
        RCLCPP_INFO(testNode_->get_logger(), "Testing - in setup");

        executor_.add_node(testNode_);

        rclcpp::Node::SharedPtr fakeTwistPub = std::make_shared<FakeTwistSubscriberNode>();
        rclcpp::Node::SharedPtr fakeOdomSub = std::make_shared<FakeOdomPublisherNode>(); 
        rclcpp::Node::SharedPtr fakeTfPub = std::make_shared<FakeTFPublisherNode>();
        rclcpp::Node::SharedPtr fakeImagePub = std::make_shared<FakeImagePublisherNode>();

        fakeNodes_.push_back(fakeTwistPub);
        fakeNodes_.push_back(fakeOdomSub);
        fakeNodes_.push_back(fakeTfPub);
        fakeNodes_.push_back(fakeImagePub);

        executor_.add_node(fakeTwistPub);
        executor_.add_node(fakeOdomSub);
        executor_.add_node(fakeTfPub);
        executor_.add_node(fakeImagePub);
       // RCLCPP_INFO(testNode_->get_logger(), "Testing - added common fakes");

        if(GetParam().pluginType == OrbitHazardManagerPluginType::Nav2){
            rclcpp::Node::SharedPtr fakeSetParamsServer = std::make_shared<FakeSetParameterServer>();
            rclcpp::Node::SharedPtr fakeNav2FollowPathServer = std::make_shared<FakeNav2FollowPathServer>();
            fakeNodes_.push_back(fakeSetParamsServer);
            fakeNodes_.push_back(fakeNav2FollowPathServer);

            executor_.add_node(fakeSetParamsServer);
            executor_.add_node(fakeNav2FollowPathServer);

            
         //   RCLCPP_INFO(testNode_->get_logger(), "Testing - added Nav2 fakes");
        }

        spinThread_= std::thread([this]() {
            executor_.spin();
        });

        RCLCPP_INFO(testNode_->get_logger(), "Testing - spin started");

        testPluginLoader_ = std::make_unique<pluginlib::ClassLoader<OrbitHazardManagerPluginBase>>(
                                    "ground_agent_on_arrival_task_plugins", 
                                    "OrbitHazardManagerPluginBase");
        RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin loader created");                                    
        testPlugin_ = testPluginLoader_->createSharedInstance(GetParam().pluginName);

        RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin created");
    }

    void TearDown() override
    {
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
    std::unique_ptr<pluginlib::ClassLoader<OrbitHazardManagerPluginBase>> testPluginLoader_;
    std::shared_ptr<OrbitHazardManagerPluginBase> testPlugin_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr nav2Subscription_;
    rclcpp::Subscription<transportMsgs::MissionStatus>::SharedPtr twistSubscription_;
};

TEST_P(OrbitHazardManagerPluginTests, OrbitHazardCompletedSuccess) {
  
    RCLCPP_INFO(testNode_->get_logger(), "Testing - started");
    std::atomic_bool messageReceived {false};
    if(GetParam().pluginType == OrbitHazardManagerPluginType::Nav2) {
        RCLCPP_INFO(testNode_->get_logger(), "Testing - started - Nav2");
        nav2Subscription_ = testNode_->create_subscription<transportMsgs::MissionCompleted>(
                                        "orbit_completed", 10, 
                                        [&](const transportMsgs::MissionCompleted& missionCompletedMsg){
                                             if(missionCompletedMsg.success)   {
                                                RCLCPP_INFO(testNode_->get_logger(), "Mission Completed!");
                                                messageReceived = true;
                                             }
                                        });
    } else {
        RCLCPP_INFO(testNode_->get_logger(), "Testing - started - Twist");
        twistSubscription_ = testNode_->create_subscription<transportMsgs::MissionStatus>(
                                        "staging_completed", 10, 
                                        [&](const transportMsgs::MissionStatus& missionStatusMsg){
                                             if(missionStatusMsg.success)   {
                                                RCLCPP_INFO(testNode_->get_logger(), 
                                                    "Staging Completed! Orbit will be tested during simulation testing");
                                                messageReceived = true;
                                             }
                                        });
        
    }

    testPlugin_->initialise(testNode_);
    RCLCPP_INFO(testNode_->get_logger(), "Testing - plugin initialised");
    auto request = std::make_shared<OrbitHazard::Request>();
    request->hazard_type = AgentUtils::toHazardTypeString(HazardType::FIBROUS_HAZARD);
    request->estimated_hazard_diameter = 6;
    geometry_msgs::msg::Point centerPoint;
    centerPoint.x = 5.0;
    centerPoint.y = 2.0;
    centerPoint.z = 0.5;
    request->hazard_center_point = centerPoint;
    RCLCPP_INFO(testNode_->get_logger(), "Testing - request created");
    testPlugin_->doHazardOrbit(request);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(messageReceived);
}

INSTANTIATE_TEST_SUITE_P(
    OrbitHazardManagerPlugins,
    OrbitHazardManagerPluginTests,
    ::testing::Values(
        OrbitHazardManagerPluginTestData{
            "Nav2OrbitHazardManagerPlugin",
            OrbitHazardManagerPluginType::Nav2,
            "Nav2"
        },
        OrbitHazardManagerPluginTestData{
            "TwistCommanderOrbitHazardPlugin",
            OrbitHazardManagerPluginType::Twist,
            "Twist"
        }),
        [](const testing::TestParamInfo<OrbitHazardManagerPluginTestData>& info){
            return info.param.pluginTypeName;
        });        


int main(int argc, char** argv) {

    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}