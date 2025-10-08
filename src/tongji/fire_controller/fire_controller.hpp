#pragma once

#include <memory>

#include "interfaces/fire_controller.hpp"
#include "tongji/identifier/identified_armor.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController(bool auto_fire, const double& control_delay_in_second,
        const double& bullet_speed, double yaw_offset, double pitch_offset);

    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const override;
    const enumeration ::CarIDFlag GetAttackCarId() const override;

    void UpdateIdentifiedArmors(const identifier::IdentifiedArmor& armors);
    void UpdateGimbalPosition(const Eigen::Vector3d& gimbal_pos);

    void SetTimeStamp(const std::time_t& time_stamp);
    predictor::TimeStamp GetTimeStamp() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}