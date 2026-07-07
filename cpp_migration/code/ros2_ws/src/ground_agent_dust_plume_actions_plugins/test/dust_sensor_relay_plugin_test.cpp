#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "pluginlib/class_loader.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "transport_interfaces/msg/dust_sensor_result.hpp"
#include "dust_sensor_relay_plugin/dust_sensor_relay_plugin.hpp"

class FakeDustSensorRelayPublisher: public rclcpp::Node 
{
public:
    FakeDustSensorRelayPublisher(): Node("fake_dust_sensor_relay_publisher") {
        dustSensorResultPub_ = this->create_publisher<transportMsgs::DustSensorResult>("/dust/agilex_diff_drive", 10);
        pubTimer_ = this->create_wall_timer(std::chrono::seconds(1), 
                        std::bind(&FakeDustSensorRelayPublisher::publishDustSensorResult, this));
    }

private:
    rclcpp::Publisher<transportMsgs::DustSensorResult>::SharedPtr dustSensorResultPub_;
    inline static const float scalingConstant_{-0.003};

    double getMaximumOpacity(const ignition::math::Vector3d &p, 
                                            const ignition::math::Pose3d &plumePose) const
    {
        auto d = p - plumePose.Pos();
        double r = std::hypot(d.X(), d.Y());
        double z = std::fabs(d.Z());

        if(r >= radius_ || z >= height_ / 2.0)
            return 0.0;
        
        return 1.0 - (r / radius_);
    }

    void publishDustSensorResult() {
        ignition::math::Vector3d point{1.0, 2.0, 3.0};    
        ignition::math::Pose3d pose{
            1.0, 2.0, 3.0,    // x, y, z
            0.0, 0.0, 1.57    // roll, pitch, yaw (radians)
        };
        double k = getMaximumOpacity(point, pose); 

        transportMsgs::DustSensorResult dustSensorResultMsg;
        dustSensorResultMsg.pm1_0 = k * pm1Max_;
        dustSensorResultMsg.pm2_5 = k * pm2Max_;
        dustSensorResultMsg.pm10 = k * pm10Max_;
        dustSensorResultMsg.tsp = dustSensorResultMsg.pm10;
        dustSensorResultMsg.opacity = k * (1.0 - std::exp(scalingConstant_ * pm2Max_));
        dustSensorResultMsg.wind_speed = windSpeed_;
        dustSensorResultMsg.wind_direction = windDirDeg_;
        dustSensorResultMsg.humidity = humidity_;
        dustSensorResultMsg.temperature = temperature_;

        dustSensorResultPub_->publish(dustSensorResultMsg);
        
    }

    rclcpp::TimerBase::SharedPtr pubTimer_;
    double radius_{5.0}, height_{3.0};
    double pm1Max_{220.0}, pm2Max_{550.0}, pm10Max_{900.0};
    double windSpeed_{3.8}, windDirDeg_{120.0}, temperature_{28.0}, humidity_{38.0};
};

TEST(DustSensorRelayPluginTests, PublishSuccess)
{
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto fakePub = std::make_shared<FakeDustSensorRelayPublisher>();
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