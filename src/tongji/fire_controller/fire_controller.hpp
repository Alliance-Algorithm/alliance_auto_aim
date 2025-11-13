#pragma once

#include <memory>

#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController(const std::string& config_path,
        std::shared_ptr<interfaces::ICarState> const& state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> const& live_target_manager);
    ~FireController();

    data ::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const override;
    enumeration ::CarIDFlag GetAttackCarId() const override;

    void UpdateGimbalPosition(const double& gimbal_yaw);

    auto GetArmorsToView() -> std::shared_ptr<interfaces::IArmorInGimbalControl>;

    FireController(const FireController&)                = delete;
    FireController& operator=(const FireController&)     = delete;
    FireController(FireController&&) noexcept            = default;
    FireController& operator=(FireController&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
