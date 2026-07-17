#include "path_planner_plugin/nav2_path_planner_plugin.hpp"
#include <stdexcept>
#include <chrono>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agent_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

Nav2PathPlannerPlugin::Nav2PathPlannerPlugin() 
			: PathPlannerPlugin("Nav2PathPlannerPlugin")
{
}

void Nav2PathPlannerPlugin::initialise(const rclcpp::Node::SharedPtr &node)
{
	coreInitialise(node);
	if(auto node = node_.lock()) {
		computePathToPoseClient_ = rclcpp_action::create_client<ComputePathToPose>(node, "compute_path_to_pose");
	} else {
		auto msg = "Some exception occurred - Node not available";
        RCLCPP_ERROR(logger_, msg);
	}
}

void Nav2PathPlannerPlugin::onComputePathGoalResponse(GoalHandle::SharedPtr goal_handle)
{
	if(!goal_handle) {
		RCLCPP_WARN_STREAM(logger_, "ComputePathToGoal goal rejected");
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "Compute Path to pose accepted, waiting for completion");

	computePathToPoseClient_->async_get_result(goal_handle, 
								std::bind(&Nav2PathPlannerPlugin::computePathResult, this, std::placeholders::_1));
	
}

void Nav2PathPlannerPlugin::computePathResult(const GoalHandle::WrappedResult &result)
{
	
	auto path = result.result->path;
	RCLCPP_INFO_STREAM(logger_, "Compute Path to pose returned " << path.poses.size() << " poses");
	auto waypoints = decimate(path, logger_);

//	RCLCPP_INFO_STREAM(logger_, "Down-sampled to " << waypoints.size() << " poses");

	if(!waypoints.empty() && AgentUtils::getPoseDistance(waypoints[0], path.poses[0]) < FIRSTPOSEDIFFERENCE)
		waypoints.erase(waypoints.begin());

	RCLCPP_INFO_STREAM(logger_, "Down-sampled to " << waypoints.size() << " poses");

	for (auto &w : waypoints)
	{
		w.header.frame_id = path.header.frame_id;
		w.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
	}
	
	auto resultMsg = std::make_shared<RequestPathPlan::Result>();
	resultMsg->success = true;
	resultMsg->message = "Waypoints sent";
	resultMsg->waypoints = waypoints;

	rclcpp_action::ClientGoalHandle<RequestPathPlan>::WrappedResult wrappedResult;
	wrappedResult.code = rclcpp_action::ResultCode::SUCCEEDED;
	wrappedResult.result = resultMsg;

	if(requestOptions_.result_callback) 
		requestOptions_.result_callback(wrappedResult);
	
}

geometry_msgs::msg::PoseStamped Nav2PathPlannerPlugin::getPose(geometry_msgs::msg::Pose goalPose)
{
	geometry_msgs::msg::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x = goalPose.position.x;
	pose.pose.position.y = goalPose.position.y;
	pose.pose.position.z = goalPose.position.z;
	pose.pose.orientation.x = goalPose.orientation.x;
	pose.pose.orientation.y = goalPose.orientation.y;
	pose.pose.orientation.z = goalPose.orientation.z;
	pose.pose.orientation.w = goalPose.orientation.w;

	return pose;
}

std::vector<PoseStamped> Nav2PathPlannerPlugin::decimate(nav_msgs::msg::Path path, rclcpp::Logger logger)
{
	std::vector<PoseStamped> prunedWayPoints;
	std::vector<PoseStamped> poses  = path.poses;
    if(poses.empty()) {
		return prunedWayPoints;
	}

	RCLCPP_INFO_STREAM(logger, "Poses: " << poses.size());	

	prunedWayPoints.push_back(path.poses.front());
	RCLCPP_INFO_STREAM(logger, "Pruned wayponts with 1st: " << poses.size());	
	for(auto itr = poses.begin() + 1; itr != poses.end(); ++itr) {
		if(AgentUtils::getPoseDistance(poses.back(), *itr) >= SPACING)
			prunedWayPoints.push_back(*itr);
	}

	RCLCPP_INFO_STREAM(logger, "Pruned waypoints: " << prunedWayPoints.size());	

	if(AgentUtils::getPoseDistance(prunedWayPoints.back(), poses.back()) > EUCLID) 
			prunedWayPoints.push_back(poses.back());

	RCLCPP_INFO_STREAM(logger, "[2]Pruned waypoints: "  << prunedWayPoints.size());	
	return prunedWayPoints;
}

void Nav2PathPlannerPlugin::plan_path(RequestPathPlan::Goal request, SendGoalOptions options)
{
	requestOptions_ = options;
	try {
		if(!computePathToPoseClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			RCLCPP_INFO_STREAM(logger_, "Waiting for Nav2 action servers …");
			return;
		}

		auto goalMsg = ComputePathToPose::Goal();
		goalMsg.goal = getPose(request.goal);
		goalMsg.use_start = false;
		rclcpp_action::Client<ComputePathToPose>::SendGoalOptions computePathToPoseOptions;
		computePathToPoseOptions.goal_response_callback = std::bind(&Nav2PathPlannerPlugin::onComputePathGoalResponse,
																	this, std::placeholders::_1);
		computePathToPoseClient_->async_send_goal(goalMsg, computePathToPoseOptions);

	}catch(const std::runtime_error& ex) {
		RCLCPP_ERROR_STREAM(logger_, "Exception occured whilst trying to plan path to hazard" << ex.what());
	}

}
PLUGINLIB_EXPORT_CLASS(Nav2PathPlannerPlugin, PathPlannerPluginBase)