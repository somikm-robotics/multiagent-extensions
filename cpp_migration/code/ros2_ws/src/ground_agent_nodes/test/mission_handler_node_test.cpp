#include <atomic>
#include <chrono>
#include <memory>
#include <gtest/gtest.h>

#include "mission_handler_node.hpp"
#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"
#include "test_helpers/fake_image_publisher_node.hpp"
#include "test_helpers/fake_tf_publisher_node.hpp"
#include "test_helpers/nav2/fake_nav2_follow_path_server.hpp"
#include "test_helpers/nav2/fake_nav2_set_parameter_server.hpp"
#include "test_helpers/fake_dust_sensor_replay_publisher_node.hpp"
#include "test_helpers/nav2/fake_nav2_follow_waypoints_server.hpp"
#include "test_helpers/nav2/fake_nav2_compute_path_to_pose_server.hpp"
#include "test_helpers/test_utils.hpp"

#include "dust_plume_actions_node.hpp"
#include "arrival_task_node.hpp"

#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/srv/density_regressor.hpp"
#include "transport_interfaces/msg/base_return.hpp"
#include "transport_interfaces/msg/hazard_pose.hpp"

using namespace std::chrono_literals;

using DensityRegressor = transport_interfaces::srv::DensityRegressor;

class MissionHandlerNodeTest: public ::testing::Test {
protected:
    void SetUp() override {
        underTestNode_ = std::make_shared<MissionHandlerNode>();
        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - in setup");
        executor_.add_node(underTestNode_);

        arrivalTaskNode_ = std::make_shared<ArrivalTaskNode>();
        executor_.add_node(arrivalTaskNode_);

        // common fakes
        auto fakeComputePathToPoseServer = std::make_shared<FakeComputePathToPoseServer>();
        auto fakeFollowWayPointsServer = std::make_shared<FakeFollowWaypointsServer>();
        auto fakeImagePubNode = std::make_shared<FakeImagePublisherNode>();
        auto fakeOdomPubNode = std::make_shared<FakeOdomPublisherNode>();
        auto fakeTwistSubNode = std::make_shared<FakeTwistSubscriberNode>();
        auto fakeTFPubNode = std::make_shared<FakeTFPublisherNode>();
        
        fakeNodes_.push_back(fakeComputePathToPoseServer);
        fakeNodes_.push_back(fakeFollowWayPointsServer);
        fakeNodes_.push_back(fakeImagePubNode);
        fakeNodes_.push_back(fakeOdomPubNode);
        fakeNodes_.push_back(fakeTwistSubNode);
        fakeNodes_.push_back(fakeTFPubNode);

        executor_.add_node(fakeComputePathToPoseServer);
        executor_.add_node(fakeFollowWayPointsServer);
        executor_.add_node(fakeImagePubNode);
        executor_.add_node(fakeOdomPubNode);
        executor_.add_node(fakeTwistSubNode);
        executor_.add_node(fakeTFPubNode);

        spinThread_= std::thread([this]() {
            executor_.spin();
        });

        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - spin started");
        hazardDetectedPub_ = underTestNode_->create_publisher<transportMsgs::HazardPose>("hazard_detected", 10);
 
        hazardDetectedMsg_.estimated_hazard_diameter = 6.0;
        geometry_msgs::msg::Point centerPoint;
        centerPoint.x = 5.0;
        centerPoint.y = 2.0;
        centerPoint.z = 0.5;
        hazardDetectedMsg_.hazard_center_point = centerPoint;
        geometry_msgs::msg::Pose hazardPose;
        hazardPose.position.x = 1.0;
        hazardPose.position.y = 2.0;
        hazardPose.orientation.w = 1.0;
        hazardDetectedMsg_.pose = hazardPose;

        baseReturnSub_ = underTestNode_->create_subscription<transportMsgs::BaseReturn>(
                                            "return_to_base_status", 10, [&]
                                                    (const transportMsgs::BaseReturn& returnMsg){
                                                if(returnMsg.success) {
                                                    RCLCPP_INFO(underTestNode_-> get_logger(), returnMsg.message.c_str());
                                                    messageReceived_ = true;
                                                }
                                            });
    }

    void initialiseForDustPlumeActions() {
        fakeDustSensorRelayPubNode_ = std::make_shared<FakeDustSensorRelayPublisherNode>();
        fakeNodes_.push_back(fakeDustSensorRelayPubNode_);
        executor_.add_node(fakeDustSensorRelayPubNode_);

        dustPlumeActionsNode_ = std::make_shared<DustPlumeActionsNode>();
        executor_.add_node(dustPlumeActionsNode_);
        densityRegressorServiceProcess_ = TestUtils::createDensityRegressorProcess();
        dustPlumeActionsNode_->initialise();
    }

    void performTest() {

        arrivalTaskNode_->initialise();
        underTestNode_->initialise();   
        hazardDetectedPub_->publish(hazardDetectedMsg_);

        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !messageReceived_ &&
            std::chrono::steady_clock::now() - start < std::chrono::seconds(300))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
        RCLCPP_INFO(underTestNode_->get_logger(), "Elapsed: %lld s", 
                            static_cast<long long>(elapsed.count()));

        EXPECT_TRUE(messageReceived_);
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
        executor_.remove_node(arrivalTaskNode_);
        executor_.remove_node(underTestNode_);
        RCLCPP_INFO(underTestNode_->get_logger(), "Testing - completed Tear down");
        underTestNode_.reset();
    }

    std::shared_ptr<MissionHandlerNode> underTestNode_;
    std::thread spinThread_;
    std::vector<rclcpp::Node::SharedPtr> fakeNodes_;

    rclcpp::executors::MultiThreadedExecutor executor_;
    rclcpp::Subscription<transportMsgs::BaseReturn>::SharedPtr baseReturnSub_;
    std::atomic_bool messageReceived_{false};
    rclcpp::Publisher<transportMsgs::HazardPose>::SharedPtr hazardDetectedPub_;
    transportMsgs::HazardPose hazardDetectedMsg_;
    std::unique_ptr<bp::child> densityRegressorServiceProcess_;
    std::shared_ptr<DustPlumeActionsNode> dustPlumeActionsNode_;
    rclcpp::Node::SharedPtr fakeSetParameterServer_;
    rclcpp::Node::SharedPtr fakeNav2FollowPathServer_;
    rclcpp::Node::SharedPtr fakeDustSensorRelayPubNode_;
    std::shared_ptr<ArrivalTaskNode> arrivalTaskNode_;
};

TEST_F(MissionHandlerNodeTest, FibrousHazardSuccess) {

    RCLCPP_INFO(underTestNode_->get_logger(), "Testing started");
    auto imagesFolderPath = AgentUtils::getImageOutputFolderPath();
    auto imagePath = std::filesystem::path(imagesFolderPath) / "initial_inspection";

    AgentUtils::clearFolder(imagePath);

    hazardDetectedMsg_.hazard_type = AgentUtils::toHazardTypeString(HazardType::FIBROUS_HAZARD);

    fakeSetParameterServer_ = std::make_shared<FakeSetParameterServer>();
    fakeNav2FollowPathServer_ = std::make_shared<FakeNav2FollowPathServer>();

    fakeNodes_.push_back(fakeSetParameterServer_);
    fakeNodes_.push_back(fakeNav2FollowPathServer_);

    executor_.add_node(fakeSetParameterServer_);
    executor_.add_node(fakeNav2FollowPathServer_);

    performTest();
    
}

TEST_F(MissionHandlerNodeTest, DustPlumeHazardSuccess) {

    initialiseForDustPlumeActions();

    hazardDetectedMsg_.hazard_type = AgentUtils::toHazardTypeString(HazardType::DUST_PLUME);

    performTest();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    testing::InitGoogleTest(&argc, argv);

    
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();

    return result;
}