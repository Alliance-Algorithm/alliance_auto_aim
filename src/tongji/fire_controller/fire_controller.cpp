#include "fire_controller.hpp"

#include <memory>
#include <utility>

#include "../identifier/identified_armor.hpp"
#include "../predictor/live_target_manager/live_target_manager.hpp"
#include "../predictor/target_snapshot_manager/target_snapshot_manager.hpp"
#include "../state_machine/state_machine.hpp"
#include "data/fire_control.hpp"
#include "fire_decision.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::fire_control {

using TargetSnapshotManager = predictor::TargetSnapshotManager;
using StateMachine          = state_machine::StateMachine;
using IdentifiedArmor       = identifier::IdentifiedArmor;
using CarIDFlag             = enumeration::CarIDFlag;
using LiveTargetManager     = predictor::LiveTargetManager;
using TimeStamp             = time_stamp::TimeStamp;

class FireController::Impl {
public:
    Impl(bool auto_fire, const double& control_delay_in_second, const double& bullet_speed,
        double yaw_offset, double pitch_offset,
        std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : control_delay_(control_delay_in_second)
        , bullet_speed_(bullet_speed)
        , fire_decision_(std::make_unique<FireDecision>(auto_fire))
        , state_machine_(state_machine)
        , live_target_manager_(std::move(live_target_manager)) { }

    // TODO:std::time_t
    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const {

        if (!identified_armors_ || !fire_decision_ || !state_machine_ || !live_target_manager_)
            return { .fire_allowance = false };

        auto converged_cars   = state_machine_->GetAllowdToFires();
        auto snapshot_manager = live_target_manager_->GetPredictor(converged_cars);
        if (!snapshot_manager)
            return data::FireControl { .time_stamp = time_stamp_.GetTimeStamp(),
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false };

        auto armors_in_gimbal_control = snapshot_manager->Predictor(control_delay_);
        allowed_target_id_            = state_machine_->GetAllowdToFires();

        auto target_gimbal_spacing =
            armors_in_gimbal_control->GetArmors(allowed_target_id_).front();

        auto gimbal_command =
            std::dynamic_pointer_cast<TargetSnapshotManager>(snapshot_manager)->GetGimbalCommand();

        auto fire_command =
            fire_decision_->ShouldFire(gimbal_command, target_gimbal_spacing.position);
        firable_ = fire_command;

        data::FireControl result;
        result.fire_allowance = fire_command;
        result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;
        result.time_stamp = time_stamp_.GetTimeStamp();
        return result;
    }

    const CarIDFlag GetAttackCarId() const {
        if (firable_) { }
        return allowed_target_id_;
    }

    void Update(std::shared_ptr<interfaces::IArmorInImage> armors, const double& gimbal_yaw) {
        time_stamp_.SetTimeStamp(std::time(nullptr));
        UpdateIdentifiedArmor(armors);
        UpdateGimbalPosition(gimbal_yaw);
    }

private:
    void UpdateIdentifiedArmor(std::shared_ptr<interfaces::IArmorInImage> armors) {
        identified_armors_ = armors;
    }
    void UpdateGimbalPosition(const double& gimbal_yaw) { gimbal_yaw_ = gimbal_yaw; };
    TimeStamp GetTimeStamp() const { return time_stamp_; }

    double gimbal_yaw_;
    double control_delay_;
    double bullet_speed_;

    mutable CarIDFlag allowed_target_id_;
    mutable double firable_ { false };

    std::shared_ptr<interfaces::IArmorInImage> identified_armors_;

    std::unique_ptr<FireDecision> fire_decision_;
    time_stamp::TimeStamp time_stamp_ { std::time(nullptr) };

    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
};

FireController::FireController(std::shared_ptr<interfaces::ICarState> state_machine, bool auto_fire,
    const double& control_delay_in_second, const double& bullet_speed, double yaw_offset,
    double pitch_offset, std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
    : pimpl_(std::make_unique<Impl>(auto_fire, control_delay_in_second, bullet_speed, yaw_offset,
          pitch_offset, state_machine, live_target_manager)) { }

const data ::FireControl FireController::CalculateTarget(const std ::time_t& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

const CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }

}