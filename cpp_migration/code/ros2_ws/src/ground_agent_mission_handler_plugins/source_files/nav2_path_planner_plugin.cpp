#include "path_planner_plugin/nav2_path_planner_plugin.hpp"
#include <stdexcept>
#include <chrono>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agent_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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
		auto message = "ComputePathToGoal goal rejected";
		RCLCPP_WARN_STREAM(logger_, message);
		sendResult(message, rclcpp_action::ResultCode::CANCELED, false);
		return;
	}	
	RCLCPP_INFO_STREAM(logger_, "Compute Path to pose accepted, waiting for completion");

	computePathToPoseClient_->async_get_result(goal_handle, 
								std::bind(&Nav2PathPlannerPlugin::computePathResult, this, std::placeholders::_1));
	
}

void Nav2PathPlannerPlugin::computePathResult(const GoalHandle::WrappedResult &result)
{
	bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
					 result.result && result.result->error_code == 0;
	
	if(!succeeded) {
		auto message = "Waypoints not sent - Error in processing - " 
						+ (result.result ?  std::to_string(result.result->error_code) : "");

		sendResult(message, rclcpp_action::ResultCode::ABORTED, false);
		return;
	}

	auto path = result.result->path;
	
	RCLCPP_INFO_STREAM(logger_, "Compute Path to pose returned " << path.poses.size() << " poses");

	auto returnedWayoints = path.poses;
	for (auto &w: returnedWayoints) {
		RCLCPP_DEBUG_STREAM(logger_, "x: " << w.pose.position.x << "y: " << w.pose.position.y << " orientation w: " << w.pose.orientation.w);
	}

	auto waypoints = decimate(path, logger_);

	RCLCPP_INFO_STREAM(logger_, "Down-sampled to before erase start" << waypoints.size() << " poses");

	if(!waypoints.empty() && AgentUtils::getPoseDistance(waypoints[0], path.poses[0]) < FIRSTPOSEDIFFERENCE)
		waypoints.erase(waypoints.begin());

	RCLCPP_INFO_STREAM(logger_, "Down-sampled to " << waypoints.size() << " poses");

	// compute orientation
	setOrientation(waypoints);

	for (auto &w : waypoints)
	{
		w.header.frame_id = path.header.frame_id;
		w.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
	}

	for (auto &waypoint: waypoints) {
		RCLCPP_DEBUG_STREAM(logger_, "x: " << waypoint.pose.position.x << " y: " 
						<< waypoint.pose.position.y << " w: " << waypoint.pose.orientation.w);
	}

	sendResult("Waypoints sent", rclcpp_action::ResultCode::SUCCEEDED,true, waypoints);	
	
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

	RCLCPP_DEBUG_STREAM(logger, "Poses: " << poses.size());	

	prunedWayPoints.push_back(path.poses.front());
	RCLCPP_DEBUG_STREAM(logger, "Pruned wayponts with 1st: " << poses.size());	
	for(auto itr = poses.begin() + 1; itr != poses.end(); ++itr) {
		if(AgentUtils::getPoseDistance(prunedWayPoints.back(), *itr) >= SPACING)
			prunedWayPoints.push_back(*itr);
	}

	RCLCPP_DEBUG_STREAM(logger, "Pruned waypoints: " << prunedWayPoints.size());	

	if(AgentUtils::getPoseDistance(prunedWayPoints.back(), poses.back()) > EUCLID) 
			prunedWayPoints.push_back(poses.back());

	
	RCLCPP_DEBUG_STREAM(logger, "[2]Pruned waypoints: "  << prunedWayPoints.size());	
	return prunedWayPoints;
}

void Nav2PathPlannerPlugin::setOrientation(std::vector<PoseStamped> &poses)
{
	// robot's pos in travel order
	tf2::Quaternion qLast;
	for(size_t i = 0; i + 1 < poses.size(); i++){
		double dx = poses[i+1].pose.position.x - poses[i].pose.position.x;
		double dy = poses[i+1].pose.position.y - poses[i].pose.position.y;
		auto q = AgentUtils::yawtoQuat(std::atan2(dy, dx));
		poses[i].pose.orientation = tf2::toMsg(q);
		qLast = q;
	}

	poses.back().pose.orientation = tf2::toMsg(qLast);
}

void Nav2PathPlannerPlugin::sendResult(std::string message, rclcpp_action::ResultCode code, bool success, 
										std::vector<geometry_msgs::msg::PoseStamped> waypoints)
{
	rclcpp_action::ClientGoalHandle<RequestPathPlan>::WrappedResult wrappedResult;		
	auto resultMsg = std::make_shared<RequestPathPlan::Result>();

	wrappedResult.code = code;
	resultMsg->message = message;
	resultMsg->success = success;
	resultMsg->waypoints = waypoints;
	wrappedResult.result = resultMsg;
	
	if(requestOptions_.result_callback) 
		requestOptions_.result_callback(wrappedResult);
}

void Nav2PathPlannerPlugin::planPath(RequestPathPlan::Goal request, SendGoalOptions options)
{
	isReturn_ = request.is_return;
	requestOptions_ = options;
	try {
		if(!computePathToPoseClient_->wait_for_action_server(std::chrono::duration<float>(serviceTimeout_))) {
			auto message = "Waiting for Nav2 action servers …timed out";
			RCLCPP_INFO_STREAM(logger_, message);
			sendResult(message, rclcpp_action::ResultCode::ABORTED, false);

			return;
		}

		auto goalMsg = ComputePathToPose::Goal();
		goalMsg.goal = getPose(request.goal);
		RCLCPP_DEBUG_STREAM(logger_, "Goal: x: " << goalMsg.goal.pose.position.x << " y: " << goalMsg.goal.pose.position.y);
		goalMsg.use_start = false;
		rclcpp_action::Client<ComputePathToPose>::SendGoalOptions computePathToPoseOptions;
		computePathToPoseOptions.goal_response_callback = std::bind(&Nav2PathPlannerPlugin::onComputePathGoalResponse,
																	this, std::placeholders::_1);
		computePathToPoseClient_->async_send_goal(goalMsg, computePathToPoseOptions);

	}catch(const std::runtime_error& ex) {
		std::string msg = std::string("Exception occured whilst trying to plan path to hazard - ") + ex.what();
		RCLCPP_ERROR_STREAM(logger_, msg);
		sendResult(msg, rclcpp_action::ResultCode::ABORTED, false);
	}

}
PLUGINLIB_EXPORT_CLASS(Nav2PathPlannerPlugin, PathPlannerPluginBase)