#include "fire_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <memory>

#include <opencv2/core/cvdef.h>

#include "enum/car_id.hpp"
#include "tongji/fire_controller/aim_solver.hpp"
#include "tongji/fire_controller/fire_decision.hpp"
#include "tongji/fire_controller/tracker.hpp"
#include "tongji/identifier/identified_armor.hpp"
#include "tongji/predictor/live_target_manager.hpp"
#include "tongji/predictor/target_snapshot_manager.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "tongji/state_machine/state_machine.hpp"

namespace world_exe::tongji::fire_control {

using TargetSnapshotManager = predictor::TargetSnapshotManager;
using StateMachine          = state_machine::StateMachine;
using IdentifiedArmor       = identifier::IdentifiedArmor;
using CarIDFlag             = enumeration::CarIDFlag;
using LiveTargetManager     = predictor::LiveTargetManager;
using TimeStamp             = predictor::TimeStamp;

class FireController::Impl {
public:
    Impl(std::shared_ptr<StateMachine> state_machine, bool auto_fire,
        const double& control_delay_in_second, const double& bullet_speed, double yaw_offset,
        double pitch_offset)
        : control_delay_(control_delay_in_second)
        , bullet_speed_(bullet_speed)
        , tracker_(std::make_unique<DefaultTracker>())
        , aim_solver_(std::make_unique<AimingSolver>(bullet_speed, yaw_offset, pitch_offset))
        , fire_decision_(std::make_unique<FireDecision>(auto_fire))
        , live_target_manager_(std::make_shared<LiveTargetManager>(state_machine)) { }

    // TODO:std::time_t
    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const {
        if (!identified_armors_ || !tracker_ || !aim_solver_ || !fire_decision_
            || !live_target_manager_)
            return { .fire_allowance = false };

        auto snapshot_manager = std::dynamic_pointer_cast<TargetSnapshotManager>(
            live_target_manager_->GetPredictor(live_target_manager_->GetLiveTargetIDs()));
        auto snapshot = tracker_->SelectTrackingTarget(*identified_armors_, snapshot_manager);

        bool found = (snapshot != nullptr);
        tracker_->UpdateState(found);
        if (!found) return { .fire_allowance = false };

        aim_solver_->UpdateSnapshot(std::move(snapshot));

        const auto& aim_solution = aim_solver_->SolveAimSolution(static_cast<double>(
            time_duration)); // TODO:std::time_t should not be converted directly to double type
        if (!aim_solution.valid) return { .fire_allowance = false };

        const Eigen::Vector3d target_pos = aim_solution.aim_point.head<3>();
        const bool fireable = fire_decision_->ShouldFire(aim_solution, target_pos, gimbal_pos_);

        if (fireable) attacked_cars_ = snapshot->GetID();

        return { .time_stamp = time_stamp_.GetTimeStamp(),
            .gimbal_dir      = Eigen::Vector3d(aim_solution.yaw, aim_solution.pitch, 0),
            .fire_allowance  = fireable };
    }

    const CarIDFlag GetAttackCarId() const { return attacked_cars_; }

    void UpdateIdentifiedArmors(const IdentifiedArmor& armors) { identified_armors_ = armors; }
    void UpdateGimbalPosition(const Eigen::Vector3d& gimbal_pos) { gimbal_pos_ = gimbal_pos; };
    void SetTimeStamp(const std::time_t& time_stamp) { time_stamp_.SetTimeStamp(time_stamp); }
    TimeStamp GetTimeStamp() const { return time_stamp_; }

private:
    Eigen::Vector3d gimbal_pos_ { Eigen::Vector3d::Zero() };
    double control_delay_;
    double bullet_speed_;

    mutable CarIDFlag attacked_cars_ {
        CarIDFlag::None
    }; // TODO:Mutable should not be used to break const encapsulation

    mutable std::optional<IdentifiedArmor> identified_armors_;
    std::unique_ptr<DefaultTracker> tracker_;
    std::shared_ptr<predictor::LiveTargetManager> live_target_manager_;
    std::unique_ptr<AimingSolver> aim_solver_;
    std::unique_ptr<FireDecision> fire_decision_;

    predictor::TimeStamp time_stamp_ { std::time(nullptr) };
};

FireController::FireController(std::shared_ptr<StateMachine> state_machine, bool auto_fire,
    const double& control_delay_in_second, const double& bullet_speed, double yaw_offset,
    double pitch_offset)
    : pimpl_(std::make_unique<Impl>(state_machine, auto_fire, control_delay_in_second, bullet_speed,
          yaw_offset, pitch_offset)) { }

const data ::FireControl FireController::CalculateTarget(const std ::time_t& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

const CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }

void FireController::UpdateIdentifiedArmors(const IdentifiedArmor& armors) {
    return pimpl_->UpdateIdentifiedArmors(armors);
}

void FireController::UpdateGimbalPosition(const Eigen::Vector3d& gimbal_pos) {
    return pimpl_->UpdateGimbalPosition(gimbal_pos);
};

}