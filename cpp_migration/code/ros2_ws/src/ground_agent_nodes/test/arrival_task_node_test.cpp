#include <atomic>
#include <chrono>
#include <memory>
#include <gtest/gtest.h>

#include "arrival_task_node.hpp"
#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"
#include "test_helpers/fake_image_publisher_node.hpp"
#include "test_helpers/fake_tf_publisher_node.hpp"
#include "test_helpers/nav2/fake_nav2_follow_path_server.hpp"
#include "test_helpers/nav2/fake_nav2_set_parameter_server.hpp"
#include "test_helpers/fake_dust_sensor_replay_publisher_node.hpp"
#include "test_helpers/test_utils.hpp"
#include "dust_plume_actions_node.hpp"

#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/srv/density_regressor.hpp"

using namespace std::chrono_literals;

using DensityRegressor = transport_interfaces::srv::DensityRegressor;


class ArrivalTaskNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        underTestNode_ = std::make_shared<ArrivalTaskNode>();

        executor_.add_node(underTestNode_);

        // common fake
        auto fakeImagePubNode = std::make_shared<FakeImagePublisherNode>();
        auto fakeOdomPubNode = std::make_shared<FakeOdomPublisherNode>();
        auto fakeTwistSubNode = std::make_shared<FakeTwistSubscriberNode>();
        auto fakeTFPubNode = std::make_shared<FakeTFPublisherNode>();

        fakeNodes_.push_back(fakeImagePubNode);
        fakeNodes_.push_back(fakeOdomPubNode);
        fakeNodes_.push_back(fakeTwistSubNode);
        fakeNodes_.push_back(fakeTFPubNode);

        executor_.add_node(fakeImagePubNode);
        executor_.add_node(fakeOdomPubNode);
        executor_.add_node(fakeTwistSubNode);
        executor_.add_node(fakeTFPubNode);

        spinThread_= std::thread([this]() {
            executor_.spin();
        });

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - spin started");

        onArrivalTaskPub_ = underTestNode_->create_publisher<transportMsgs::OnArrivalTask>("on_arrival_task", 10);

        missionStatusSub_= underTestNode_->create_subscription<transportMsgs::MissionStatus>(
                                            "on_arrival_task_mission_status", 10, [&](
                                                const transportMsgs::MissionStatus missionStatusMsg) {
                                                    if(missionStatusMsg.success) {
                                                        RCLCPP_INFO(underTestNode_->get_logger(), missionStatusMsg.message.c_str());
                                                        messageReceived_ = true;
                                                    }
                                            });
        
    }   
    
    void TearDown() override {
        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in Tear down");
        executor_.cancel();
        if(spinThread_.joinable()) {
            spinThread_.join();
        }

        if(densityRegressorServiceProcess_ && densityRegressorServiceProcess_->running()) {
            densityRegressorServiceProcess_->terminate();
            densityRegressorServiceProcess_->wait();
        }

        for(auto& fakeNode: fakeNodes_)  {
            executor_.remove_node(fakeNode);
            fakeNode.reset();
        }

        if(dustPlumeActionsNode_) {
            executor_.remove_node(dustPlumeActionsNode_);
            dustPlumeActionsNode_.reset();
        }
        //RCLCPP_INFO(underTestNode_->get_logger(), "Testing - removed dust plume actions node");

        executor_.remove_node(underTestNode_);
        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - completed Tear down");
        underTestNode_.reset();
    }

    std::shared_ptr<ArrivalTaskNode> underTestNode_;
    std::thread spinThread_;
    std::vector<rclcpp::Node::SharedPtr> fakeNodes_;

    rclcpp::executors::MultiThreadedExecutor executor_;
    rclcpp::Subscription<transportMsgs::MissionStatus>::SharedPtr missionStatusSub_;
    std::atomic_bool messageReceived_{false}; 
    rclcpp::Publisher<transportMsgs::OnArrivalTask>::SharedPtr onArrivalTaskPub_;
    std::unique_ptr<bp::child> densityRegressorServiceProcess_;
    rclcpp::Node::SharedPtr fakeSetParamsServer_;
    rclcpp::Node::SharedPtr fakeNav2FollowPathServer_;
    rclcpp::Node::SharedPtr fakeDustSensorRelayPubNode_;
    std::shared_ptr<DustPlumeActionsNode> dustPlumeActionsNode_;
};

TEST_F(ArrivalTaskNodeTest, InitialInspectionCompletedSucess) {
    
    auto imagesFolderPath = AgentUtils::getImageOutputFolderPath();
    auto imagePath = std::filesystem::path(imagesFolderPath) / "initial_inspection";

    AgentUtils::clearFolder(imagePath);

    underTestNode_->initialise();   

    transportMsgs::OnArrivalTask onArrivalTaskMsg;
    onArrivalTaskMsg.hazard_type = AgentUtils::toHazardTypeString(HazardType::FIBROUS_HAZARD);
    onArrivalTaskMsg.estimated_hazard_diameter = 6.0;
    onArrivalTaskMsg.task_type = AgentUtils::toTaskTypeStr(TaskType::INITIAL_INSPECTION);
    onArrivalTaskPub_->publish(onArrivalTaskMsg);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !messageReceived_ &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(messageReceived_);
}

TEST_F(ArrivalTaskNodeTest, AsbestosOrbitCompletedSucess) {
   
    // fakes
    fakeSetParamsServer_ = std::make_shared<FakeSetParameterServer>();
    fakeNav2FollowPathServer_= std::make_shared<FakeNav2FollowPathServer>();
    
    executor_.add_node(fakeSetParamsServer_);
    executor_.add_node(fakeNav2FollowPathServer_);

    fakeNodes_.push_back(fakeSetParamsServer_);
    fakeNodes_.push_back(fakeNav2FollowPathServer_);

    underTestNode_->initialise();   
    transportMsgs::OnArrivalTask onArrivalTaskMsg;
    onArrivalTaskMsg.hazard_type = AgentUtils::toHazardTypeString(HazardType::FIBROUS_HAZARD);
    onArrivalTaskMsg.estimated_hazard_diameter = 6.0;
    geometry_msgs::msg::Point centerPoint;
    centerPoint.x = 5.0;
    centerPoint.y = 2.0;
    centerPoint.z = 0.5;
    onArrivalTaskMsg.hazard_center_point = centerPoint;
    onArrivalTaskMsg.task_type = AgentUtils::toTaskTypeStr(TaskType::ASBESTOS_ANALYSIS);
    onArrivalTaskPub_->publish(onArrivalTaskMsg);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !messageReceived_ &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(messageReceived_);
}

TEST_F(ArrivalTaskNodeTest, DustPlumeActionCompletedSucess) {
   
    // fakes
    fakeDustSensorRelayPubNode_ = std::make_shared<FakeDustSensorRelayPublisherNode>();
    fakeNodes_.push_back(fakeDustSensorRelayPubNode_);
    executor_.add_node(fakeDustSensorRelayPubNode_);

    dustPlumeActionsNode_ = std::make_shared<DustPlumeActionsNode>();
    executor_.add_node(dustPlumeActionsNode_);
    
    densityRegressorServiceProcess_ = TestUtils::createDensityRegressorProcess();
    dustPlumeActionsNode_->initialise();
    
    underTestNode_->initialise();   
    transportMsgs::OnArrivalTask onArrivalTaskMsg;
    onArrivalTaskMsg.hazard_type = AgentUtils::toHazardTypeString(HazardType::DUST_PLUME);
    onArrivalTaskPub_->publish(onArrivalTaskMsg);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !messageReceived_ &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(messageReceived_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    testing::InitGoogleTest(&argc, argv);

    
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}