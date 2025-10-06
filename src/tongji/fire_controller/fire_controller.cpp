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
#include "tongji/predictor/target_snapshot_manager.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "tongji/state_machine/state_machine.hpp"

namespace world_exe::tongji::fire_control {

using TargetSnapshotManager = predictor::TargetSnapshotManager;

class FireController::Impl {
public:
    Impl(bool auto_fire, const double& control_delay_in_second, const double& bullet_speed,
        double yaw_offset, double pitch_offset)
        : control_delay_(control_delay_in_second)
        , bullet_speed_(bullet_speed)
        , tracker_(std::make_unique<DefaultTracker>())
        , predictor_(nullptr) // TODO
        , aim_solver_(std::make_unique<AimingSolver>(bullet_speed, yaw_offset, pitch_offset))
        , fire_decision_(std::make_unique<FireDecision>(auto_fire))
        , state_machine_(std::make_unique<state_machine::StateMachine>()) { }

    // TODO:std::time_t
    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const {
        if (!identified_armors_ || !tracker_ || !predictor_ || !aim_solver_ || !fire_decision_)
            return { .fire_allowance = false };

        auto snapshot = tracker_->SelectTrackingTarget(*identified_armors_, predictor_);
        bool found    = (snapshot != nullptr);
        tracker_->UpdateState(found);
        if (!found) return { .fire_allowance = false };

        aim_solver_->UpdateSnapshot(std::move(snapshot));

        const auto& aim_solution =
            aim_solver_->SolveAimSolution(static_cast<double>(time_duration)); // TODO
        if (!aim_solution.valid) return { .fire_allowance = false };

        state_machine_->Update(identified_armors_->GetDetectedIDs(), time_stamp_.GetTimeStamp());

        const auto allowed_id = state_machine_->GetAllowdToFires(); // TODO
        const auto current_id = tracker_->GetCurrentTargetID();

        const Eigen::Vector3d target_pos = aim_solution.aim_point.head<3>();
        const bool fireable = fire_decision_->ShouldFire(aim_solution, target_pos, gimbal_pos_)
            && state_machine_->IsAllowedToFire(current_id);
            
        return { .time_stamp = time_stamp_.GetTimeStamp(),
            .gimbal_dir      = Eigen::Vector3d(aim_solution.yaw, aim_solution.pitch, 0),
            .fire_allowance  = fireable };
    }

    // TODO
    const enumeration ::CarIDFlag GetAttackCarId() const {
        if (!tracker_) return enumeration::CarIDFlag::Unknow;
        return tracker_->GetCurrentTargetID();
    }

    void SetSnapshotManager(std::shared_ptr<predictor::TargetSnapshotManager> manager) {
        predictor_ = std::move(manager);
    }

    void UpdateArmors(const identifier::IdentifiedArmor& armors) { identified_armors_ = armors; }
    void UpdateGimbalPosition(const Eigen::Vector3d& gimbal_pos) { gimbal_pos_ = gimbal_pos; };
    void SetTimeStamp(const std::time_t& time_stamp) { time_stamp_.SetTimeStamp(time_stamp); }

private:
    Eigen::Vector3d gimbal_pos_ { Eigen::Vector3d::Zero() };
    double control_delay_;
    double bullet_speed_;

    mutable std::optional<identifier::IdentifiedArmor> identified_armors_;
    std::unique_ptr<DefaultTracker> tracker_;
    std::shared_ptr<predictor::TargetSnapshotManager> predictor_;
    std::unique_ptr<AimingSolver> aim_solver_;
    std::unique_ptr<FireDecision> fire_decision_;
    std::unique_ptr<state_machine::StateMachine> state_machine_;

    predictor::TimeStamp time_stamp_ { 0 };
};

FireController::FireController()
    : pimpl_() { }

const data ::FireControl FireController::CalculateTarget(const std ::time_t& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

void FireController::SetSnapshotManager(std::shared_ptr<predictor::TargetSnapshotManager> manager) {
    pimpl_->SetSnapshotManager(std::move(manager));
}

const enumeration ::CarIDFlag FireController::GetAttackCarId() const {
    return pimpl_->GetAttackCarId();
}

}