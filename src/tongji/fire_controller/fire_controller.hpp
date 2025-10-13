#pragma once

#include <memory>

#include "../time_stamp/time_stamp.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController(std::shared_ptr<interfaces::ICarState> state_machine, bool auto_fire,
        const double& control_delay_in_second, const double& bullet_speed, double yaw_offset,
        double pitch_offset, std::shared_ptr<interfaces::ITargetPredictor> live_target_manager);

    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const override;
    const enumeration ::CarIDFlag GetAttackCarId() const override;

    void Update(std::shared_ptr<interfaces::IArmorInImage> armors, const double& gimbal_yaw);

    predictor::TimeStamp GetTimeStamp() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}