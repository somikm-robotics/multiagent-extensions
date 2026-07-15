#include "test_helpers/fake_twist_subscriber_node.hpp"
#include "test_helpers/fake_odom_publisher_node.hpp"
#include "test_helpers/fake_tf_publisher_node.hpp"
#include "test_helpers/fake_image_publisher_node.hpp"
#include "test_helpers/fake_dust_sensor_replay_publisher_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

void FakeTwistSubscriberNode::twistCallback(
    const geometry_msgs::msg::Twist::SharedPtr)
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

    odomMsg.child_frame_id = "agilex_diff_drive/base_link";

    odomMsg.pose.pose.position.y = 1.0;
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

FakeTFPublisherNode::FakeTFPublisherNode() : Node("fake_tf_publisher_node")
{
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FakeTFPublisherNode::broadcast, this));
}
void FakeTFPublisherNode::broadcast()
{
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "agilex_diff_drive/base_link";

    //RCLCPP_INFO(this->get_logger(), "Testing - x_: %f", x_);
    auto x = xPositions_[currentPosCtr_];
    if(currentPosCtr_ < (static_cast<int>(xPositions_.size()) - 1))
        currentPosCtr_++;
    tf.transform.translation.x = x;
    tf.transform.translation.y = 2.0;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(tf);
}


FakeImagePublisherNode::FakeImagePublisherNode() : Node("fake_image_pubkusher_node")
{
    imagePub_ = this->create_publisher<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10);
    pubTimer_ = create_wall_timer(std::chrono::seconds(1),
                        std::bind(&FakeImagePublisherNode::publishImage, this));
}
void FakeImagePublisherNode::publishImage()
{
    //   RCLCPP_INFO(this->get_logger(), "publishing image");
        static const auto imagePath = 
            std::filesystem::path(
                ament_index_cpp::get_package_share_directory("shared_infrastructure"))
            /"images"
            /"test"
            /"test_image.jpg";
        
        // RCLCPP_INFO(this->get_logger(), "image path: %s", imagePath.c_str());

        cv::Mat cvImage = cv::imread(imagePath.string());

        if(cvImage.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read image: %s",
                            imagePath.string().c_str());
            return;
        }

        cv_bridge::CvImage cvBridgeImage;
        cvBridgeImage.header.frame_id = "camera";
        cvBridgeImage.header.stamp = this->get_clock()->now();
        cvBridgeImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvBridgeImage.image = cvImage;

        imagePub_ -> publish(*cvBridgeImage.toImageMsg());
       // RCLCPP_INFO(this->get_logger(), "published image");
}

FakeDustSensorRelayPublisherNode::FakeDustSensorRelayPublisherNode() : Node("fake_dust_sensor_relay_publisher")
{
    dustSensorResultPub_ = this->create_publisher<transportMsgs::DustSensorResult>("/dust/agilex_diff_drive", 10);
    pubTimer_ = this->create_wall_timer(std::chrono::seconds(1), 
                        std::bind(&FakeDustSensorRelayPublisherNode::publishDustSensorResult, this));
}
double FakeDustSensorRelayPublisherNode::getMaximumOpacity(const ignition::math::Vector3d &p, const ignition::math::Pose3d &plumePose) const
{
    auto d = p - plumePose.Pos();
    double r = std::hypot(d.X(), d.Y());
    double z = std::fabs(d.Z());

    if(r >= radius_ || z >= height_ / 2.0)
        return 0.0;
    
    return 1.0 - (r / radius_);
}

void FakeDustSensorRelayPublisherNode::publishDustSensorResult()
{
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
