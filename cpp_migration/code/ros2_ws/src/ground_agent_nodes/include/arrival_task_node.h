#include "rclcpp/rclcpp.hpp"

#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/msg/on_arrival_task.hpp"
#include "transport_interfaces/msg/mission_status.hpp"
#include "transport_interfaces/msg/mission_completed.hpp"

#include "transport_interfaces/srv/initial_inspection.hpp"
#include "transport_interfaces/srv/orbit_hazard.hpp"
#include "transport_interfaces/srv/plume_density.hpp"

namespace transportMsgs = transport_interfaces::msg;

using PlumeDensity = transport_interfaces::srv::PlumeDensity;
using InitialInspection = transport_interfaces::srv::InitialInspection;
using OrbitHazard = transport_interfaces::srv::OrbitHazard;

class ArrivalTaskNode : public rclcpp::Node {
private:

    //publishers
    rclcpp::Publisher<transportMsgs::MissionStatus>::SharedPtr _missionStatusPub;

    // subscriptions
    rclcpp::Subscription<transportMsgs::OnArrivalTask>::SharedPtr _onArrivalTaskStartSub;
    rclcpp::Subscription<transportMsgs::DensityEstimationResult>::SharedPtr _onDensityEstimationCompletedSub;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr _onInspectionCompletedSub;
    rclcpp::Subscription<transportMsgs::MissionCompleted>::SharedPtr _onOrbitCompletedSub;
    
    // clients
    rclcpp::Client<InitialInspection>::SharedPtr _initialInspectionClient;
    rclcpp::Client<OrbitHazard>::SharedPtr _orbitHazardClient;
    rclcpp::Client<PlumeDensity>::SharedPtr _plumeDensityClient;

    //methods
    void onArrival(transportMsgs::OnArrivalTask& arrivalTask);
    void onDensityEstimationComplete(PlumeDensity plumeDensity);
    void onOrbitComplete(rclcpp::Client<OrbitHazard>::SharedFuture);
    void onInitialInspectionComplete(transportMsgs:: MissionCompleted& inspectionCompleted);
    void publishTaskCompleteMessage(std::string& message, bool success = true);

public:
    ArrivalTaskNode();
    ~ArrivalTaskNode() = default;

};