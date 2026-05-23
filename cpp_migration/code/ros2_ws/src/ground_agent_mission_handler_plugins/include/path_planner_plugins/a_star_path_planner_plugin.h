#include "rclcpp/rclcpp.hpp"
#include <memory>




class AStarPathPlannerPlugin : public PathPlannerPlugin {


private:

public:
    AStarPathPlannerPlugin() = default;

    void plan_path(RequestPathPlan::Request request,
        RequestPathPlan::Response response) override;

    ~AStarPathPlannerPlugin() = default;
};