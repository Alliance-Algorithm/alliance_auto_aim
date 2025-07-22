#pragma once

#include "data/fire_control.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/predictor.hpp"
#include <memory>
namespace world_exe::v1::fire_control {
class TracingFireControl final : public interfaces::IFireControl {
public:
    const data::FireControl CalculateTarget(const std::time_t& time_duration) const override;
    const enumeration::CarIDFlag GetAttackCarId() const override;
    void SetArmorsInGimbalControl(const interfaces::IArmorInGimbalControl& armors);
    void SetPredictor(const interfaces::IPredictor& predictor);
    void SetTargetCarID(const enumeration::CarIDFlag& tracing_id);
    void SetTimeStamp(const time_t& time);
    TracingFireControl(
        double control_delay_in_second, double velocity_begin, double gravity = 9.81);

private:
    friend class TracingFireCalculateImpl;
    class TracingFireSelectImpl;
    std::unique_ptr<TracingFireSelectImpl> p_select_impl_;
    class TracingFireCalculateImpl;
    std::unique_ptr<TracingFireCalculateImpl> p_calc_impl_;
    time_t time_stamp_             = 0;
    time_t control_delay_          = 0;
    enumeration::CarIDFlag target_ = enumeration::CarIDFlag::None;
    const double velocity_begin_;
    const double gravity_;
    static inline const data::FireControl no_allow_ { .fire_allowance = false };
};
}