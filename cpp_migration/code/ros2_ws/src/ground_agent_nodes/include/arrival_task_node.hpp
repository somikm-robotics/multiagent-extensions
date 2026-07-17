#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "initial_inspection_plugin/initial_inspection_plugin_base.hpp"
#include "orbit_hazard_plugin/orbit_hazard_manager_plugin_base.hpp"

#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/mission_completed.hpp"
#include "transport_interfaces/msg/start_mission.hpp"

#include "transport_interfaces/srv/initial_inspection.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"
#include "transport_interfaces/srv/plume_density.hpp"

#include "shared_infrastructure/enums.hpp"
#include "agent_utils.hpp"

namespace transportMsgs = transport_interfaces::msg;

using InitialInspection = transport_interfaces::srv::InitialInspection;
using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class ArrivalTaskNode : public rclcpp::Node {
public:
    ArrivalTaskNode();
    
    void initialise();
    
    ~ArrivalTaskNode();

private:

    //publishers
    rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr missionStatusPub_;
    rclcpp::Publisher<transportMsgs::StartMission>::SharedPtr startDustPlumeActionsMissionPub_;

    // subscriptions
    rclcpp::Subscription<transportMsgs::OnArrivalTask>::SharedPtr onArrivalTaskStartSub_;
    rclcpp::Subscription<transportMsgs::MissionStatus>::SharedPtr onDustPlumeActionsCompletedSub_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr onInspectionCompletedSub_;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr onOrbitCompletedSub_;

    //methods
    void onArrival(const transportMsgs::OnArrivalTask& arrivalTaskMsg);
    void onDustPlumeActionsComplete(const transportMsgs::MissionStatus& dustPlumeActionsCompletedMsg);
    void onOrbitComplete(const transportMsgs::MissionCompleted& orbitCompletedMsg);
    void onInitialInspectionComplete(const transportMsgs:: MissionCompleted& inspectionCompletedMsg);
    void publishTaskCompleteMessage(const std::string& message, bool success = true);

    // plugins
    pluginlib::ClassLoader<InitialInspectionPluginBase> initialInspectionPluginLoader_;
    std::shared_ptr<InitialInspectionPluginBase> initialInspectionPlugin_;

    pluginlib::ClassLoader<OrbitHazardManagerPluginBase> orbitHazardPluginLoader_;
    std::shared_ptr<OrbitHazardManagerPluginBase> orbitHazardPlugin_;

    // variables
    HazardType hazardType_;
    TaskType taskType_;

};