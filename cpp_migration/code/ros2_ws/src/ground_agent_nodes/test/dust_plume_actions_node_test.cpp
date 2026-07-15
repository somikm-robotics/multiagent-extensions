#include <atomic>
#include <chrono>
#include <memory>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <boost/process.hpp>

#include "test_helpers/fake_image_publisher_node.hpp"
#include "test_helpers/fake_dust_sensor_replay_publisher_node.hpp"

#include "dust_plume_actions_node.hpp"
#include "transport_interfaces/srv/density_regressor.hpp"
#include "transport_interfaces/msg/mission_status.hpp"

using namespace std::chrono_literals;
namespace bp = boost::process;

using DensityRegressor = transport_interfaces::srv::DensityRegressor;

class DustPlumeActionsNodeTest : public ::testing::Test {

protected:

    void SetUp() override {
        underTestNode_ = std::make_shared<DustPlumeActionsNode>();

        densityRegressorServiceProcess_ = std::make_unique<bp::child>(
            bp::search_path("ros2"),
            bp::args({
                "run",
                "ai_python",
                "density_regressor_node"
            }),
            bp::std_out > stdout,
            bp::std_err > stderr);
        
        auto densityRegressorClient = underTestNode_->create_client<DensityRegressor>("density_regressor");

        ASSERT_TRUE(densityRegressorClient->wait_for_service(10s));
        RCLCPP_INFO(underTestNode_->get_logger(), "Density Regressor Service Process started");
    
        executor_.add_node(underTestNode_);
        // create fakes
        auto fakeImagePubNode = std::make_shared<FakeImagePublisherNode>();
        auto fakeDustSensorRelayPubNode = std::make_shared<FakeDustSensorRelayPublisherNode>();

        executor_.add_node(fakeImagePubNode);
        executor_.add_node(fakeDustSensorRelayPubNode);

        fakeNodes_.push_back(fakeImagePubNode);
        fakeNodes_.push_back(fakeDustSensorRelayPubNode);

        spinThread_= std::thread([this]() {
            executor_.spin();
        });

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - spin started");
    }

    void TearDown() override {

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down");
        executor_.cancel();
        if(spinThread_.joinable()) {
            spinThread_.join();
        }

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down - joined");
        if(densityRegressorServiceProcess_ && densityRegressorServiceProcess_->running()) {
            densityRegressorServiceProcess_->terminate();
            densityRegressorServiceProcess_->wait();
        }

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down - terminated");

        for(auto& fakeNode: fakeNodes_)  {
            executor_.remove_node(fakeNode);
            fakeNode.reset();
        }

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down - removed fake nodes");

        executor_.remove_node(underTestNode_);
        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down - removed test node");
        underTestNode_.reset();
       // RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down - reset test node");
    }

    std::shared_ptr<DustPlumeActionsNode> underTestNode_;
    std::thread spinThread_;
    std::vector<rclcpp::Node::SharedPtr> fakeNodes_;
    
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::unique_ptr<bp::child> densityRegressorServiceProcess_;
};

TEST_F(DustPlumeActionsNodeTest, PublishSuccessStatus) {

    RCLCPP_INFO(underTestNode_->get_logger(), "In Test");
    std::atomic_bool messageReceived{false};
    auto missionStatusPub_= underTestNode_->create_subscription<transportMsgs::MissionStatus>(
                                            "dust_plume_actions_complete", 10, [&](
                                                const transportMsgs::MissionStatus missionStatusMsg) {
                                                    if(missionStatusMsg.success) {
                                                        RCLCPP_INFO(underTestNode_->get_logger(), missionStatusMsg.message.c_str());
                                                        messageReceived = true;
                                                    }
                                            });
    
    
    underTestNode_->initialise();                        
    auto startPub = underTestNode_->create_publisher<transportMsgs::StartMission>("start_dust_plume_actions_task", 10);
    transportMsgs::StartMission startMissionMessage;

    startPub->publish(startMissionMessage);
    RCLCPP_INFO(underTestNode_->get_logger(), "Published Message");

    auto start = std::chrono::steady_clock::now();
    while(rclcpp::ok() && !messageReceived &&
            std::chrono::steady_clock::now() - start < 30s) {
        std::this_thread::sleep_for(10ms);
    }

    EXPECT_TRUE(messageReceived);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    testing::InitGoogleTest(&argc, argv);

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}