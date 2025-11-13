#include "fire_controller.hpp"

#include <chrono>
#include <memory>

#include <print>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "../identifier/identified_armor.hpp"
#include "../state_machine/state_machine.hpp"
#include "aim_solver.hpp"
#include "data/fire_control.hpp"
#include "fire_decision.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/predictor/car_predictor/car_predictor_manager.hpp"

namespace world_exe::tongji::fire_control {

using StateMachine        = state_machine::StateMachine;
using IdentifiedArmor     = identifier::IdentifiedArmor;
using CarIDFlag           = enumeration::CarIDFlag;
using CarPredictorManager = predictor ::CarPredictorManager;
using TimeStamp           = data::TimeStamp;

class FireController::Impl {
public:
    Impl(const std::string& config_path, std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : locked_target(CarIDFlag::None)
        , firable_(false)
        , aiming_solver_(std::make_unique<AimingSolver>(config_path))
        , fire_decision_(std::make_unique<FireDecision>(config_path))
        , state_machine_(std::move(state_machine))
        , live_target_manager_(std::move(live_target_manager))
        , control_delay_(100) { }

    data ::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const {
        if (!fire_decision_ || !state_machine_ || !live_target_manager_)
            return { .fire_allowance = false };

        const auto& lockable_target = state_machine_->GetAllowdToFires();

        const auto& snapshot_manager = live_target_manager_->GetPredictor(lockable_target);

        if (!snapshot_manager)
            return data::FireControl { .time_stamp = data::TimeStamp { time_stamp },
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false };
        // TODO:这里不应该指针转换
        const auto& aim_solution =
            aiming_solver_->SolveAimSolution(snapshot_manager, time_stamp, control_delay_);
        // armors_to_view_ = aiming_solver_->GetArmorsToView();

        if (!aim_solution.valid) {
            std::println("aim solution invalid ,solver failed");
            return data::FireControl { .time_stamp = time_stamp,
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false };
        }
        const auto gimbal_command = GimbalCommand { aim_solution.yaw, aim_solution.pitch };
        const auto target_pos     = Eigen::Vector3d { aim_solution.aim_point };

        auto fire_command = aim_solution.valid
            ? fire_decision_->ShouldFire(gimbal_yaw_, gimbal_command, target_pos)
            : false;
        firable_          = fire_command;

        data::FireControl result;
        // result.fire_allowance = fire_command;
        result.fire_allowance = true;

        // result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;
        result.gimbal_dir << cos(gimbal_command.yaw) * cos(gimbal_command.pitch),
            sin(gimbal_command.yaw) * cos(gimbal_command.pitch), sin(gimbal_command.pitch);
        result.time_stamp = time_stamp;
        return result;
    }

    CarIDFlag GetAttackCarId() const {
        if (firable_) return locked_target;
        return CarIDFlag::None;
    }

    void UpdateGimbalPosition(const double& gimbal_yaw) { gimbal_yaw_ = gimbal_yaw; };

    // auto GetArmorsToView() -> std::shared_ptr<interfaces::IArmorInGimbalControl> {
    //     return armors_to_view_;
    // }

private:
    std::chrono::milliseconds control_delay_;

    double gimbal_yaw_;

    CarIDFlag locked_target;
    mutable bool firable_;

    std::unique_ptr<AimingSolver> aiming_solver_;
    std::unique_ptr<FireDecision> fire_decision_;
    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;

    // mutable std::shared_ptr<interfaces::IArmorInGimbalControl> armors_to_view_;
};

FireController::FireController(const std::string& config_path,
    std::shared_ptr<interfaces::ICarState> const& state_machine,
    std::shared_ptr<interfaces::ITargetPredictor> const& live_target_manager)
    : pimpl_(std::make_unique<Impl>(config_path, state_machine, live_target_manager)) { }
FireController::~FireController() = default;

data ::FireControl FireController::CalculateTarget(data::TimeStamp const& time_stamp) const {
    return pimpl_->CalculateTarget(time_stamp);
}
CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }

void FireController::UpdateGimbalPosition(const double& gimbal_yaw) {
    return pimpl_->UpdateGimbalPosition(gimbal_yaw);
};

// auto FireController::GetArmorsToView() -> std::shared_ptr<interfaces::IArmorInGimbalControl> {
//     return pimpl_->GetArmorsToView();
// }

}
