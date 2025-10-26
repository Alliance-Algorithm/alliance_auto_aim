#include "fire_controller.hpp"

#include <chrono>
#include <memory>

#include <yaml-cpp/yaml.h>

#include "../identifier/identified_armor.hpp"
#include "../predictor/live_target_manager/live_target_manager.hpp"
#include "../predictor/target_snapshot_manager/target_snapshot_manager.hpp"
#include "../state_machine/state_machine.hpp"
#include "data/fire_control.hpp"
#include "fire_decision.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/time_stamp/time_stamp.hpp"

namespace world_exe::tongji::fire_control {

using TargetSnapshotManager = predictor::TargetSnapshotManager;
using StateMachine          = state_machine::StateMachine;
using IdentifiedArmor       = identifier::IdentifiedArmor;
using CarIDFlag             = enumeration::CarIDFlag;
using LiveTargetManager     = predictor::LiveTargetManager;
using TimeStamp             = time_stamp::TimeStamp;

class FireControllerImpl {
public:
    FireControllerImpl(const std::string& config_path,
        std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : allowed_target_id_(CarIDFlag::None)
        , firable_(false)
        , fire_decision_(std::make_unique<FireDecision>(config_path))
        , state_machine_(state_machine)
        , live_target_manager_(live_target_manager) {

        auto yaml      = YAML::LoadFile(config_path);
        control_delay_ = yaml["control_delay"].as<double>();
        bullet_speed_  = yaml["bullet_speed"].as<double>();
    }

    // TODO:std::time_t
    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const {

        if (!identified_armors_ || !fire_decision_ || !state_machine_ || !live_target_manager_)
            return { .fire_allowance = false };

        auto converged_cars   = state_machine_->GetAllowdToFires();
        auto snapshot_manager = live_target_manager_->GetPredictor(converged_cars);
        if (!snapshot_manager)
            return data::FireControl {
                .time_stamp = TimeStamp(std::chrono::steady_clock::now()).GetTimeStamp(),
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false
            };

        auto armors_in_gimbal = snapshot_manager->Predictor(control_delay_);
        allowed_target_id_    = state_machine_->GetAllowdToFires();

        auto target_gimbal_spacing = armors_in_gimbal->GetArmors(allowed_target_id_).front();

        auto gimbal_command =
            std::dynamic_pointer_cast<TargetSnapshotManager>(snapshot_manager)->GetGimbalCommand();

        auto fire_command =
            fire_decision_->ShouldFire(gimbal_yaw_,gimbal_command, target_gimbal_spacing.position);
        firable_ = fire_command;

        data::FireControl result;
        result.fire_allowance = fire_command;
        result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;
        result.time_stamp = TimeStamp(std::chrono::steady_clock::now()).GetTimeStamp();
        return result;
    }

    const CarIDFlag GetAttackCarId() const {
        if (firable_) return allowed_target_id_;
        return CarIDFlag::None;
    }

    void Update(std::shared_ptr<interfaces::IArmorInImage> armors, const double& gimbal_yaw) {
        UpdateIdentifiedArmor(armors);
        UpdateGimbalPosition(gimbal_yaw);
    }

private:
    void UpdateIdentifiedArmor(std::shared_ptr<interfaces::IArmorInImage> armors) {
        identified_armors_ = armors;
    }
    void UpdateGimbalPosition(const double& gimbal_yaw) { gimbal_yaw_ = gimbal_yaw; };

    double gimbal_yaw_;
    double control_delay_;
    double bullet_speed_;
    std::shared_ptr<interfaces::IArmorInImage> identified_armors_;

    mutable CarIDFlag allowed_target_id_;
    mutable double firable_;

    std::unique_ptr<FireDecision> fire_decision_;
    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
};

FireController::FireController(const std::string& config_path,
    std::shared_ptr<interfaces::ICarState> state_machine,
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
    : pimpl_(
          std::make_unique<FireControllerImpl>(config_path, state_machine, live_target_manager)) { }

const data ::FireControl FireController::CalculateTarget(const std ::time_t& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

const CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }

}