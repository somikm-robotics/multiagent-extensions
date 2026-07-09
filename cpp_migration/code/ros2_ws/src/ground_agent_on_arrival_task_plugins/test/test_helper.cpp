#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

void FakeTwistSubscriberNode::twistCallback(
    const geometry_msgs::msg::Twist::SharedPtr twistMsg)
{
    // nothing to do - intentionally left blank
}


FakeTwistSubscriberNode::FakeTwistSubscriberNode() : Node("fake_twist_subscriber")
{
    twistSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                                kCmdVelTopic, 10, [this](
                                const geometry_msgs::msg::Twist::SharedPtr twistMsg){
                this->twistCallback(twistMsg);
    });
}

FakeOdomPublisherNode::FakeOdomPublisherNode() : Node("fake_odom_publisher") 
{
    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pubTimer_ = this->create_wall_timer(std::chrono::seconds(1), 
                    std::bind(&FakeOdomPublisherNode::publishOdomMessage, this));
}

void FakeOdomPublisherNode::publishOdomMessage()
{
    nav_msgs::msg::Odometry odomMsg;

    odomMsg.header.frame_id = "odom";
    odomMsg.header.stamp = this->get_clock()->now();

    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = 1.0;
    odomMsg.pose.pose.position.y = 2.0;
    odomMsg.pose.pose.position.z = 0.1;

    yaw_ += incrementCtr;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    odomMsg.pose.pose.orientation.x = q.x();
    odomMsg.pose.pose.orientation.y = q.y();
    odomMsg.pose.pose.orientation.z = q.z();
    odomMsg.pose.pose.orientation.w = q.w();

    odomPub_->publish(odomMsg);
}