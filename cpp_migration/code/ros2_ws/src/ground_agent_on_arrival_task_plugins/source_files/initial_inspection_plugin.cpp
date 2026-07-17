#include "initial_inspection_plugin/initial_inspection_plugin.hpp"
#include <opencv2/imgcodecs.hpp>
#include "agent_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


InitialInspectionPlugin::InitialInspectionPlugin() 
            : logger_{rclcpp::get_logger("InitialInspectionPlugin")}
{
}

void InitialInspectionPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
    node_ = node;
    hazardType_ = HazardType::NONE;
    busy_ = false;
    spinActive_ = false;
    imagesCaptured_ = 0;

    //Publishers
    twistPub_ = node->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);
    inspectionCompletedPub_ = node->create_publisher<transportMsgs::MissionCompleted>(
                                    "inspection_complete", 10);
    //Subscriptions
    imageSub_ = node->create_subscription<sensor_msgs::msg::Image>(kCameraTopic, 10, 
        [this](const sensor_msgs::msg::Image::SharedPtr image){
        this->imageCallback(image);
    });
    odomSub_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
        [this](const nav_msgs::msg::Odometry::SharedPtr odom){
        this->odomCallback(odom);
    });

    auto imagesFolderPath = AgentUtils::getImageOutputFolderPath();
    auto imageDirPath = std::filesystem::path(imagesFolderPath)/ kImagesOutDir;
    auto currentTime = AgentUtils::getCurrentTimeFormatString("%Y%m%d_%H%M%S");
    outDir_ = AgentUtils::getFolderPathStr(imageDirPath, "captured_images_"+ currentTime);
    std::filesystem::create_directories(outDir_); 
    RCLCPP_INFO(logger_, "In service - created folder: %s", outDir_.c_str());                                                
}

std::shared_ptr<InitialInspection::Response> InitialInspectionPlugin::performInspection(std::shared_ptr<InitialInspection::Request> request)
{
    // Initialize state
    imagesCaptured_ = 0;
    spinActive_ = true;
    hazardType_ = AgentUtils::parseHazardType(request->hazard_type);
    
   
    auto msg = AgentUtils::getMessage("Capturing ", kNImages, " images -> ", 
                                        outDir_, " for ", request->hazard_type);

    auto response = std::make_shared<InitialInspection::Response>();                                        
    // Start spinning
    twist_ = geometry_msgs::msg::Twist();
    twist_.angular.z = kSpinSpeed;
    if(!hasSpinPub_) {
        if(auto node = node_.lock()) {
            spinPub_ = node->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);
        } else {
            auto msg = "Some exception occurred - Node not available";
            response->message = msg;
            RCLCPP_ERROR(logger_, msg);
            return response;
        }
    }
    
    if(spinTimer_ != nullptr && !spinTimer_->is_canceled()) {
        spinTimer_->cancel();
    }
    
    if(auto node = node_.lock())
    {
        spinTimer_ = node->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&InitialInspectionPlugin::spinPublish, this));
        RCLCPP_INFO_STREAM(logger_, "Spin started. Images will be saved as robot rotates.");
    } else {
        auto msg = "Some exception occurred - Node not available";
        response->message = msg;
        RCLCPP_ERROR(logger_, msg);
        return response;
    }

    response->message = "Spin started. Images will be saved.";
    return response;
}

void InitialInspectionPlugin::imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg)
{
   //RCLCPP_INFO(logger_, "In Image Callback: %d", imagesCaptured_);
    
    if(!spinActive_ || imagesCaptured_ >= kNImages)
        return;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*imageMsg, sensor_msgs::image_encodings::BGR8);
    auto img = cv_ptr->image;
   // RCLCPP_INFO(logger_, "In Image Callback - created image");
    
    if(!currentYaw_.has_value())
        return;

   //RCLCPP_INFO(logger_, "In Image Callback - current yaw: %f", currentYaw_.value());
   // RCLCPP_INFO(logger_, "In Image Callback - last yaw: %s", lastYaw_.has_value() ? "True" : "False");
    if(!lastYaw_.has_value()) {
        lastYaw_ = currentYaw_.value();
     //   RCLCPP_INFO(logger_, "In Image Callback - last yaw: %f", lastYaw_.value());
        saveImage(img);
        return;
    }

    // Compute angle difference (wrap-around safe)
    auto yaw_diff = std::abs(std::fmod(currentYaw_.value() - lastYaw_.value() + M_PI, 2 * M_PI) - M_PI);
    
    //RCLCPP_INFO(logger_, "In Image Callback - yaw diff: %f, kAnglePerImage %f", yaw_diff, kAnglePerImage);
    
    if(yaw_diff >= kAnglePerImage) {
        lastYaw_ = currentYaw_.value();
        saveImage(img);
    }
}

void InitialInspectionPlugin::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    //auto quat = odomMsg->pose.pose.orientation;
    //RCLCPP_INFO_STREAM(logger_, "In odom Callback 1 " << quat.w);
    //RCLCPP_INFO_STREAM(logger_, "In odom Callback 1 x: " << quat.x << " y: " << quat.y 
      //                              << " z: " << quat.z << " w: " << quat.w);
    //Convert quaternion to yaw (Euler Z)
    // auto siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    // auto cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    // RCLCPP_INFO_STREAM(logger_, "In odom Callback " << siny_cosp << " " << cosy_cosp );

    // auto currentYaw = atan2(siny_cosp, cosy_cosp);
    currentYaw_ = AgentUtils::yawFromPose((*odomMsg).pose.pose);
    // if(currentYaw_.has_value()) {
    //     RCLCPP_INFO(logger_, "In odom Callback - set current yaw : %f", currentYaw_.value());
    // }
}

void InitialInspectionPlugin::spinPublish()
{
    if(spinActive_) {
        spinPub_->publish(twist_);
    } else {
        spinPub_->publish(geometry_msgs::msg::Twist()); // send zero velocity
        spinTimer_->cancel();
    }
}

void InitialInspectionPlugin::saveImage(const cv::Mat& image)
{
    auto fName = AgentUtils::getMessage("frame_", std::setw(3), std::setfill('0') ,  imagesCaptured_ + 1, ".png");
    RCLCPP_INFO(logger_, "In save image fname: %s", fName.c_str());
    auto fullPath = AgentUtils::getFolderPathStr(outDir_, fName);
    RCLCPP_INFO(logger_, "In save image - full path: %s", fullPath.c_str());
    
    cv::imwrite(fullPath, image);
    ++imagesCaptured_;
    RCLCPP_INFO(logger_, "saved image: %d", imagesCaptured_);
    if(imagesCaptured_ >= kNImages) {
        stopSpin();
        transportMsgs::MissionCompleted msg;
        msg.success = true;
        inspectionCompletedPub_->publish(msg);
    }
}

void InitialInspectionPlugin::stopSpin()
{
    spinActive_ = false;
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = 0.0;
    if(hasSpinPub_) 
        spinPub_->publish(twist);
    if(spinTimer_ != nullptr && !spinTimer_->is_canceled())
        spinTimer_->cancel();
    //std::string message("Captured " << imagesCaptured_ << " images for " << hazardType_);
    auto message = AgentUtils::getMessage("Captured ", imagesCaptured_, " images for ", AgentUtils::toHazardTypeString(hazardType_));
    RCLCPP_INFO_STREAM(logger_, message);
    
}
PLUGINLIB_EXPORT_CLASS(InitialInspectionPlugin, InitialInspectionPluginBase)
