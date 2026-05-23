#include "rclcpp/rclcpp.hpp"

#include "transport_interfaces/msg/hazard_pose.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/navigation_result.hpp"
#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/base_return.hpp"

#include "transport_interfaces/srv/override_mission.hpp"
#include "transport_interfaces/srv/path_plan_result.hpp"
#include "transport_interfaces/srv/request_path_plan.hpp"
#include "transport_interfaces/srv/navigate_request.hpp"

#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace transportMsgs = transport_interfaces::msg;
using OverrideMission = transport_interfaces::srv::OverrideMission;
using PathPlanResult = transport_interfaces::srv::PathPlanResult;
using RequestPathPlan = transport_interfaces::srv::RequestPathPlan;
using NavigateRequest = transport_interfaces::srv::NavigateRequest;

class MissionHandlerNode : public rclcpp::Node {
    private:
        //publishers
        rclcpp::Publisher<transportMsgs::OnArrivalTask>
                ::SharedPtr _onArrivalTaskStartPub;
        rclcpp::Publisher<transportMsgs::BaseReturn>::SharedPtr _baseReturnPub;
        rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr _intermediadyMissionStatusPub;

        // subscriptions
        rclcpp::Subscription<transportMsgs::HazardPose>::SharedPtr _hazardDetectedSub;
        rclcpp::Subscription<transportMsgs::NavigationResult>::SharedPtr _navigationResultSub;
        rclcpp::Subscription<transportMsgs::OnArrivalTask>::SharedPtr _onArrivalTaskCompletionSub;

        //services
        rclcpp::Service<OverrideMission>::SharedPtr _overrideMissionService;
        rclcpp::Service<PathPlanResult>::SharedPtr _pathPlanResultService;

        // clients
        rclcpp::Client<RequestPathPlan>::SharedPtr _requestPathPlanClient;
        rclcpp::Client<NavigateRequest>::SharedPtr _navigateRequestClient;

        // subscription callbacks
        void hazardDetectedCallback(const transportMsgs::HazardPose& hazardPose);
        void reachedTargetCallback(const transportMsgs::NavigationResult& result);
        void onArrivalTaskCompletion(transportMsgs::OnArrivalTask arrivalTaskMessage);

        // client - future response handlers
        void handleNavigationResponse(rclcpp::Client<NavigateRequest>::SharedFuture future);
        void handlePathPlanResponse(rclcpp::Client<RequestPathPlan>::SharedFuture future);

        // methods
        void checkOverrideStatus();
        void publishIntermediaryStatus(std::string& message, const bool toggleCommand);
        geometry_msgs::msg::Pose& getReturnPose();
        void setOrbitGoal();
        void requestPathForNavigation();
        void sendNavigationCommand(std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
        void waitForApprovalOrTimeout();
        void publishOnArrivalTask();
        void setNextTaskType();
        void startReturnToBase(transportMsgs::OnArrivalTask arrivalTaskMessage);
        void publishReturnToBaseStatus();

    public:
        MissionHandlerNode();

        // service methods
        void handleOverride(std::shared_ptr<OverrideMission::Request> request, 
            std::shared_ptr<OverrideMission::Response> response);
        
        void handlePathResult(std::shared_ptr<PathPlanResult::Request> request, 
            std::shared_ptr<PathPlanResult::Response> response);


        ~MissionHandlerNode() = default;
        

};