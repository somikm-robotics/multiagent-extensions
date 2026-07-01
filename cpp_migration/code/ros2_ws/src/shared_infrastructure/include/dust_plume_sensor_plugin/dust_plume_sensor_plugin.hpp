#pragma once

#include <ignition/gazebo/System.hh>
#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "transport_interfaces/msg/dust_sensor_result.hpp"

namespace transportMsgs = transport_interfaces::msg;
namespace ig = ignition::gazebo;

class DustPlumeSensorPlugin
        : public ig::System,
          public ig::ISystemConfigure,
          public ig::ISystemPreUpdate,
          public ig::ISystemPostUpdate
{
private:
  // constants
  inline static const float scalingConstant_{-0.003};

  // publishers
  rclcpp::Publisher<transportMsgs::DustSensorResult>::SharedPtr dustSensorResultPub_;

    rclcpp::Node::SharedPtr node_{nullptr};
    
    //parameters
    double radius_{5.0}, height_{3.0};
    double pm1Max_{220.0}, pm2Max_{550.0}, pm10Max_{900.0};
    double windSpeed_{3.8}, windDirDeg_{120.0}, temperature_{28.0}, humidity_{38.0};
    std::string targetName_;
  
    // variables
    bool enabled_{true};

    // cached entities
    ig::Entity plumeEntity_{ig::kNullEntity};
    ig::Entity targetEntity_{ig::kNullEntity};

    // methods
    double getMaximumOpacity(const ignition::math::Vector3d &p, const ignition::math::Pose3d& plumePose) const;

public:
     /* --------- ISystemConfigure (called once, plume model entity) ---- */
    void Configure(const ig::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ig::EntityComponentManager &,
                   ig::EventManager &) override;

    /* --------- Lazy ROS init + entity resolve ------------------------ */
    void PreUpdate(const ig::UpdateInfo &,
                   ig::EntityComponentManager &ecm) override;

    /* --------- Publish dust packet every step ------------------------ */
    void PostUpdate(const ig::UpdateInfo &,
                    const ig::EntityComponentManager &ecm) override;

};
