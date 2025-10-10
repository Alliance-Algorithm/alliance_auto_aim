#pragma once

#include <memory>

#include "interfaces/armor_in_image.hpp"
#include "interfaces/fire_controller.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "tongji/state_machine/state_machine.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController(std::shared_ptr<state_machine::StateMachine> state_machine, bool auto_fire,
        const double& control_delay_in_second, const double& bullet_speed, double yaw_offset,
        double pitch_offset);

    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const override;
    const enumeration ::CarIDFlag GetAttackCarId() const override;

    void UpdateIdentifiedArmors(std::shared_ptr<interfaces::IArmorInImage> armors);
    void UpdateGimbalPosition(const double& gimbal_yaw);

    predictor::TimeStamp GetTimeStamp() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}