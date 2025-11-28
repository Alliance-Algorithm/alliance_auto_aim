#include "fire_controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <memory>

#include <utility>
#include <yaml-cpp/yaml.h>

#include "../identifier/identified_armor.hpp"
#include "../state_machine/state_machine.hpp"
#include "aim_solver.hpp"
#include "data/fire_control.hpp"
#include "data/time_stamped.hpp"
#include "fire_decision.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/fire_controller/planner/planner.hpp"
#include "tongji/predictor/car_predictor/car_predictor_manager.hpp"

namespace world_exe::tongji::fire_control {

using Planner             = planner::Planner;
using StateMachine        = state_machine::StateMachine;
using IdentifiedArmor     = identifier::IdentifiedArmor;
using CarIDFlag           = enumeration::CarIDFlag;
using CarPredictorManager = predictor ::CarPredictorManager;
using TimeStamp           = data::TimeStamp;

class FireController::Impl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Impl(const std::string& config_path, std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : aiming_solver_(std::make_unique<AimingSolver>(config_path))
        , state_machine_(std::move(state_machine))
        , live_target_manager_(std::move(live_target_manager))
        , fire_decision_(std::make_unique<FireDecision>(config_path)) { }

    data ::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const {
        if (!state_machine_ || !live_target_manager_) return { .fire_allowance = false };

        const auto& lockable_target = state_machine_->GetAllowdToFires();

        const auto& snapshot_manager = live_target_manager_->GetPredictor(lockable_target);

        if (!snapshot_manager)
            return data::FireControl { .time_stamp = data::TimeStamp { time_stamp },
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false };
        // TODO:这里不应该指针转换
        // std::println("calculate time:{}", time_stamp.to_seconds());
        const auto& aim_solution = aiming_solver_->SolveAimSolution(
            snapshot_manager, transform_gimbal2muzzle_, time_stamp, control_delay_);

        if (!aim_solution.valid) {
            // std::println("aim solution invalid ");
            throw std::runtime_error("No valid aim point!");
            // return data::FireControl { .time_stamp = time_stamp,
            //     .gimbal_dir =
            //     Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
            //     .fire_allowance = false };
        }

        const auto gimbal_command = GimbalCommand { aim_solution.yaw, aim_solution.pitch };

        const auto target_pos = Eigen::Vector3d { aim_solution.aim_point };

        auto ypr          = transform_gimbal2muzzle_.inverse().rotation().eulerAngles(2, 1, 0);
        auto gimbal_yaw   = ypr(0);
        auto gimbal_pitch = ypr(1);
        auto gimbal_roll  = ypr(2);
        std::cout << "yaw:" << gimbal_yaw * 57.3 << " pitch:" << gimbal_pitch * 57.3
                  << " roll:" << gimbal_roll * 57.3 << std::endl;

        std::cout << "command yaw:" << gimbal_command.yaw * 57.3
                  << "  pitch:" << gimbal_command.pitch * 57.3 << "du" << std::endl;

        // auto fire_valid = fire_decision_->ShouldFire(gimbal_yaw, gimbal_command, target_pos);
        // auto fire_valid = true;

        // if (!fire_valid) {
        //     result.fire_allowance = false;
        //     firable_              = false;
        //     result.gimbal_dir     = Eigen::Vector3d::Zero();
        //     std::cout << "forbidden fire" << std::endl;
        //     return result;
        // }

        firable_ = true;

        data::FireControl result;
        result.time_stamp     = time_stamp;
        result.fire_allowance = true;
        result.gimbal_dir << cos(gimbal_command.yaw) * cos(gimbal_command.pitch),
            sin(gimbal_command.yaw) * cos(gimbal_command.pitch), sin(gimbal_command.pitch);
        return result;
    }

    CarIDFlag GetAttackCarId() const {
        if (firable_) return locked_target_;
        return CarIDFlag::None;
    }

    void SetGimbal2Muzzle(Eigen::Affine3d const& transform_gimbal2muzzle) {
        transform_gimbal2muzzle_ = transform_gimbal2muzzle;
    }

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> GetArmorsSnapshot() {
        return aiming_solver_->GetArmorsSnapshot();
    }

private:
    // auto CalculateTrajectories(data::TimeStamp const& time_stamp) -> planner::Trajectory {
    //     const auto& lockable_target  = state_machine_->GetAllowdToFires();
    //     const auto& snapshot_manager = live_target_manager_->GetPredictor(lockable_target);

    //     if (snapshot_manager) return planner::Trajectory::Zero();

    //     const auto& aim_solution = aiming_solver_->SolveAimSolution(
    //         snapshot_manager, transform_gimbal2muzzle_, time_stamp, control_delay_);
    //     if (!aim_solution.valid) return planner::Trajectory::Zero();

    //     planner::Trajectory trajectories;
    // }

    std::chrono::milliseconds control_delay_ { 100 };

    CarIDFlag locked_target_ { CarIDFlag::None };
    mutable bool firable_ { false };

    Eigen::Affine3d transform_gimbal2muzzle_ { Eigen ::Affine3d::Identity() };

    std::unique_ptr<AimingSolver> aiming_solver_;
    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
    std::unique_ptr<FireDecision> fire_decision_;
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

void FireController::SetGimbal2Muzzle(Eigen::Affine3d const& transform_gimbal2muzzle) {
    return pimpl_->SetGimbal2Muzzle(transform_gimbal2muzzle);
}

std ::shared_ptr<interfaces ::IArmorInGimbalControl> FireController::GetArmorsSnapshot() {
    return pimpl_->GetArmorsSnapshot();
}

}
