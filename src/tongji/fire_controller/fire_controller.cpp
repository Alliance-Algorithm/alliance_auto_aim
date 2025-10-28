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

namespace world_exe::tongji::fire_control {

using TargetSnapshotManager = predictor::TargetSnapshotManager;
using StateMachine          = state_machine::StateMachine;
using IdentifiedArmor       = identifier::IdentifiedArmor;
using CarIDFlag             = enumeration::CarIDFlag;
using LiveTargetManager     = predictor::LiveTargetManager;
using TimeStamp             = data::TimeStamp;

class FireController::Impl {
public:
    Impl(const std::string& config_path, std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : locked_target(CarIDFlag::None)
        , firable_(false)
        , fire_decision_(std::make_unique<FireDecision>(config_path))
        , state_machine_(state_machine)
        , live_target_manager_(live_target_manager) {

        auto yaml      = YAML::LoadFile(config_path);
        control_delay_ = std::chrono::seconds{static_cast<long>(yaml["control_delay"].as<double>() * 1e9)};
    }

    //  TODO:time_duration 没有使用，详见std::shared_ptr<interfaces::IArmorInGimbalControl>
    //  Predictor(conststd::time_t&time_stamp) const这个接口的注释

    /*
    从这个接口：std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(const
    std::time_t&time_stamp) const 已经可以得到std::shared_ptr<interfaces::IArmorInGimbalControl>了
    接下来是要 据此得到最终的控制指令data ::FireControl

    为了得到最终的控制指令，data ::FireControl，需要对GimbalCommand进行判断，
    当GimbalCommand不突变 且 云台位于合适位置时，返回有效指令

    因为在这个函数中，从std::shared_ptr<interfaces::IArmorInGimbalControl>中选出了开火的对象，
    需要保存用于作为这个接口const CarIDFlag GetAttackCarId() const的返回值，所以只好破坏const约束了

    - rep: 也许改个名字就好了
    */

    const data ::FireControl CalculateTarget(const std::chrono::seconds& time_from_tracker_timepoint) const {

        if (!fire_decision_ || !state_machine_ || !live_target_manager_)
            return { .fire_allowance = false };

        auto converged_cars   = state_machine_->GetAllowdToFires();
        auto snapshot_manager = live_target_manager_->GetPredictor(converged_cars);
        if (!snapshot_manager)
            return data::FireControl {
                .time_stamp = data::TimeStamp{time_from_tracker_timepoint},
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false
            };

        // - TODO:接口语义不明，此函数传入的参数有待修正
        // - rep: 这里你需要算的不是程序执行时候的时间，而是预计的未来的某个时间的装甲板坐标
        //      btw, 不计算飞行时间的话，获取这个predictor是为了什么，
        //           为了击中未来的装甲板，需要使用某种算法去获取击中时候的装甲板位姿，然后反推云台位置
        // FIXME: 火控系统错误
        auto armors_in_gimbal = snapshot_manager->Predictor(time_from_tracker_timepoint + control_delay_);
        auto lockable_target = state_machine_->GetAllowdToFires();
        
        // 逻辑真的是选择所有可以击打的装甲板的第一个吗，这里有问题
        auto target_gimbal_spacing = armors_in_gimbal->GetArmors(lockable_target).front();

        locked_target =target_gimbal_spacing.id; 

        // FIXME: 不要偷偷的指针转换
        // 牵连太多，临时不好修
        auto gimbal_command =
            std::dynamic_pointer_cast<TargetSnapshotManager>(snapshot_manager)->GetGimbalCommand();

        auto fire_command =
            fire_decision_->ShouldFire(gimbal_yaw_, gimbal_command, target_gimbal_spacing.position);
        firable_ = fire_command;

        data::FireControl result;
        result.fire_allowance = fire_command;
        result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;
        result.time_stamp = data::TimeStamp{std::chrono::steady_clock::now().time_since_epoch()};
        return result;
    }

    /*
    - 感觉这个和状态机那边的GetAllowdToFires()有点重复了
    - rep: 不重复的，锁定和可锁定的差别（好名字，用了）
    */
    const CarIDFlag GetAttackCarId() const {
        if (firable_) return locked_target;
        return CarIDFlag::None;
    }

    void UpdateGimbalPosition(const double& gimbal_yaw) { gimbal_yaw_ = gimbal_yaw; };

private:
    double gimbal_yaw_;
    std::chrono::seconds control_delay_;

    mutable CarIDFlag locked_target;
    mutable double firable_;

    std::unique_ptr<FireDecision> fire_decision_;
    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
};

FireController::FireController(const std::string& config_path,
    std::shared_ptr<interfaces::ICarState> state_machine,
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
    : pimpl_(std::make_unique<Impl>(config_path, state_machine, live_target_manager)) { }
FireController::~FireController() = default;
const data ::FireControl FireController::CalculateTarget(const std::chrono::seconds& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

const CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }
void FireController::UpdateGimbalPosition(const double& gimbal_yaw) {
    return pimpl_->UpdateGimbalPosition(gimbal_yaw);
};

}