#include "dust_plume_sensor_plugin/dust_plume_sensor_plugin.hpp"

#include <ignition/plugin/Register.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

double DustPlumeSensorPlugin::getMaximumOpacity(const ignition::math::Vector3d &p, 
                                            const ignition::math::Pose3d &plumePose) const
{
    auto d = p - plumePose.Pos();
    double r = std::hypot(d.X(), d.Y());
    double z = std::fabs(d.Z());

    if(r >= radius_ || z >= height_ / 2.0)
        return 0.0;
    
    return 1.0 - (r / radius_);
}

void DustPlumeSensorPlugin::Configure(const ig::Entity &_entity,
                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                      ig::EntityComponentManager &,
                                      ig::EventManager &)
{
    plumeEntity_ = _entity; // this plume model

    if(!_sdf->HasElement("target_model")) {
        ignerr << "[DustPlumeSensorPlugin] <target_model> tag is required\n";
        enabled_ = false;
        return;
    }

    targetName_ = _sdf->Get<std::string>("target_model");

    _sdf->Get<double>("radius", radius_, radius_);
    _sdf->Get<double>("height", height_, height_);
    _sdf->Get<double>("pm1_0_max", pm1Max_, pm1Max_);
    _sdf->Get<double>("pm2_5_max", pm2Max_, pm2Max_);
    _sdf->Get<double>("pm10_max", pm10Max_, pm10Max_);
    _sdf->Get<double>("wind_speed", windSpeed_, windSpeed_);
    _sdf->Get<double>("wind_direction", windDirDeg_, windDirDeg_);
    _sdf->Get<double>("humidity", humidity_, humidity_);
    _sdf->Get<double>("temperature", temperature_, temperature_);
}

void DustPlumeSensorPlugin::PreUpdate(const ig::UpdateInfo &, ig::EntityComponentManager &ecm)
{
    // early-out if the plugin was disabled in Configure()
    if(!this->enabled_)
        return;

    // 1. Lazy ROS node + publisher initialisation (GasSensor style)
    if(!this->node_) {
        if(!rclcpp::ok()) {  // ensure single ROS context
            int argc = 1;
            const char *argv[] = {"dust_plume_sensor"};
            try {
                rclcpp::init(argc, const_cast<char **>(argv));

            }catch(const std::runtime_error& ex) {
                ignerr << "[DustPlumeSensorPlugin] ROS init failed: "
                       << ex.what() << std::endl;
                throw; // abort world load – no silent failure
            }
        }
        this->node_ = rclcpp::Node::make_shared("dust_plume_sensor");
        this->dustSensorResultPub_ = node_->create_publisher<transportMsgs::DustSensorResult>("/dust/" + targetName_, 10);
        ignmsg << "[DustPlumeSensorPlugin] ROS node initialised; "
               << "publishing on /dust/" << targetName_ << std::endl;
    }
    // 2. Allow any ROS callbacks to run this sim tick 
    rclcpp::spin_some(node_);

    // 3. Resolve the robot entity once
    if(targetEntity_ == ig::kNullEntity) {
        ecm.Each<ig::components::Name>(
            [this](const ig::Entity &ent, const ig::components::Name *name) -> bool {
                if(name && name->Data() == targetName_) {
                    targetEntity_ = ent;
                    return false;  // found → stop iterating
                }
                return true; // keep searching
            });
    }
 }

void DustPlumeSensorPlugin::PostUpdate(const ig::UpdateInfo &, const ig::EntityComponentManager &ecm)
{
    if(!enabled_ || !node_ || targetEntity_ == ig::kNullEntity)
        return;

    auto plumePose = ecm.Component<ig::components::Pose>(plumeEntity_);
    auto robotPose = ecm.Component<ig::components::Pose>(targetEntity_);

    if(!plumePose || !robotPose) 
        return;

    double k = getMaximumOpacity(robotPose->Data().Pos(), plumePose->Data()); 

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

// register
IGNITION_ADD_PLUGIN(
    DustPlumeSensorPlugin,
    ig::System,
    DustPlumeSensorPlugin::ISystemConfigure,
    DustPlumeSensorPlugin::ISystemPreUpdate,
    DustPlumeSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DustPlumeSensorPlugin, "dust_plume_sensor")