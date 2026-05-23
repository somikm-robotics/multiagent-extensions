#include "transport_interfaces/msg/density_estimation_result.hpp"
#include "transport_interfaces/srv/plume_density.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <memory>

using PlumeDensity = transport_interfaces::srv::PlumeDensity;
namespace transportMsgs = transport_interfaces::msg;

enum class SeverityLevel: int { Low = 1, Moderate, High, Very_High, Severe};

class DustPlumeDensityEstimationNode : public rclcpp::Node {

    private:
        rclcpp::Publisher<transportMsgs::DensityEstimationResult>::SharedPtr _completionPub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSub;
        rclcpp::Service<PlumeDensity>::SharedPtr _plumeDensityService;

        inline static const std::string CAMERA_TOPIC {"/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image"};
        
        SeverityLevel& computeSeverityLevel(double density);
        void imageCallback(const sensor_msgs::msg::Image& img);
    public:
        DustPlumeDensityEstimationNode();

        void estimatePlumeDensity(std::shared_ptr<PlumeDensity::Request> request, 
            std::shared_ptr<PlumeDensity::Response> response);

        ~DustPlumeDensityEstimationNode();
};


