#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"

# include "orbit_hazard_plugin/nav2_orbit_hazard_manager_plugin.hpp"

class FakeNav2FollowPathServer: public rclcpp::Node 
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

    FakeNav2FollowPathServer() : Node("fake_follow_path_server")
    {
        server_ = rclcpp_action::create_server<FollowPath>(
            this, "/follow_path",
            std::bind(&FakeNav2FollowPathServer::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeNav2FollowPathServer::handleCancel, this,
                std::placeholders::_1),
            std::bind(&FakeNav2FollowPathServer::handleAccepted, this,
                std::placeholders::_1));
    }

private:
    rclcpp_action::Server<FollowPath>::SharedPtr server_;
    
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowPath::Goal>) 
    {
        RCLCPP_INFO(this->get_logger(), "In handle goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "In handle cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goalHandle)
    {
        RCLCPP_INFO(this->get_logger(), "In handle accepted");

      //  RCLCPP_INFO(this->get_logger(), "handle accepted - creating result");

        auto result = std::make_shared<FollowPath::Result>();
        
        //result->missed_waypoints = {};

       // RCLCPP_INFO(this->get_logger(), "handle accepted - created result");
        
       
       // RCLCPP_INFO(this->get_logger(), "handle accepted - set params");
        goalHandle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "handle accepted - done");

    }
};

class FakeNav2SetParameterServer : public rclcpp::Node {

public:
    FakeNav2SetParameterServer() : Node("fake_nav2_set_parameter_server") {
        fakeService_ = this->create_service<SetParameters>("/controller_server/set_parameters",
                                                    std::bind(&FakeNav2SetParameterServer::handleRequest,
                                                                this, std::placeholders::_1, std::placeholders::_2));
    }    
    
private:
    rclcpp::Service<SetParameters>::SharedPtr fakeService_;

    void handleRequest(std::shared_ptr<SetParameters::Request> request, 
                          std::shared_ptr< SetParameters::Response> response) {
        response->results.resize(request->parameters.size());

        for(auto& result : response->results) {
            result.successful =  true;
            result.reason = "";
        }
    }
};

TEST(Nav2OrbitHazardManagerPluginTests, OrbitHazardSuccess) {
  
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto fakeTwistPub = std::make_shared<geometry_msgs::msg::Twist>();
    auto fakeOdomSub = std::make_shared<nav_msgs::msg::Odometry>();
    auto fakeSetParamsServer = std::make_shared<FakeNav2SetParameterServer>();
    auto fakeNav2FollowPath = std::make_shared<FakeNav2FollowPathServer>();
}