#include "initial_inspection_plugin.h"

InitialInspectionPlugin::InitialInspectionPlugin()
{
}

void InitialInspectionPlugin::initialise(const rclcpp::Node::SharedPtr& node)
{
}

void InitialInspectionPlugin::perform_inspection(InitialInspection::Request request, InitialInspection::Response response)
{
}

void InitialInspectionPlugin::imageCallback(const sensor_msgs::msg::Image& image)
{
}

void InitialInspectionPlugin::odomCallback(const nav_msgs::msg::Odometry& image)
{
}

void InitialInspectionPlugin::spinPublish()
{
}

void InitialInspectionPlugin::saveImage(const sensor_msgs::msg::Image& image)
{
}

void InitialInspectionPlugin::stopSpin()
{
}



