#include "auto_aim_system.hpp"

#include <chrono>
#include <memory>

#include "../v1/sync/syncer.hpp"
#include "core/event_bus.hpp"
#include "data/predictor_update_package.hpp"
#include "modules/fire_controller/fire_controller.hpp"
#include "modules/predictor/car_predictor/car_predictor_manager.hpp"
#include "modules/solver/solver.hpp"
#include "modules/state_machine/state_machine.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include "v1/identifier/identifier.hpp"

namespace world_exe::tongji {
using namespace std::chrono;

class AutoAimSystem::Impl {
public:
    Impl(const bool& debug)
        : debug(debug)
        , config_path_("../../configs/example.yaml") {

        identifier_ = std::make_unique<v1::identifier::Identifier>(
            parameters::ParamsForSystemV1::szu_model_path(),
            parameters::ParamsForSystemV1::device(), parameters::HikCameraProfile::get_width(),
            parameters::HikCameraProfile::get_height());
        pnp_solver_          = std::make_unique<solver::Solver>();
        live_target_manager_ = std::make_shared<predictor::CarPredictorManager>(config_path_);
        state_machine_       = std::make_shared<state_machine::StateMachine>();
        fire_controller_     = std::make_unique<fire_control::FireController>(
            config_path_, state_machine_, live_target_manager_);
        time_stamp_ = std::chrono::steady_clock::now();
        syncer_     = std::make_unique<world_exe::v1::Syncer>(seconds(2), 6e-6);

        core::EventBus::Subscript<cv::Mat>(parameters::ParamsForSystemV1::raw_image_event,
            [this](const auto& mat) { Solve(mat); });
        core::EventBus::Subscript<data::CameraGimbalMuzzleSyncData>(
            parameters::ParamsForSystemV1::camera_capture_transforms,
            [this](const auto& pkg) { SetTransfroms(pkg); });
    }

    auto Solve(const cv::Mat& raw) -> void {
        const auto& [armors_in_image, flag] = identifier_->identify(raw);
        if (flag == enumeration::ArmorIdFlag::None) return;
        state_machine_->Update(armors_in_image,
            std::chrono::duration_cast<milliseconds>(
                std::chrono::steady_clock::now() - time_stamp_));

        // 这里使用 any_clock::now 也可以，但是时间系统的转换和同步我希望是单独的部分
        const auto& [pack, check] = syncer_->get_data(armors_in_image->GetTimeStamp());
        if (!check) return;

        const auto R_camera2gimbal = pack.camera_to_gimbal.rotation();
        const auto t_camera2gimbal = pack.camera_to_gimbal.translation();

        pnp_solver_->SetCamera2Gimbal(R_camera2gimbal, t_camera2gimbal);
        const auto& armors_in_camera = pnp_solver_->SolvePnp(armors_in_image);

        auto combined = std::make_shared<data::PredictorUpdatePackage>(pack, armors_in_camera);

        live_target_manager_->Update(combined);

        state_machine_ = std::make_shared<state_machine::StateMachine>();

        const auto target_id = state_machine_->GetAllowdToFires();

        const auto gimbal_yaw = R_camera2gimbal.eulerAngles(2, 1, 0)[0];
        fire_controller_->UpdateGimbalPosition(gimbal_yaw);

        /// 这里应该有一个线程进行稳定的输出之类的
        /// 轨迹规划器没有实现，先不管

        core::EventBus::Publish<data::FireControl>(
            parameters::ParamsForSystemV1::fire_control_event, GetControlCommand());
    }

    void SetTransfroms(const data::CameraGimbalMuzzleSyncData& data) { syncer_->set_data(data); }

    data::FireControl GetControlCommand() {
        fire_controller_->GetAttackCarId();
        return fire_controller_->CalculateTarget(
            std::chrono::duration_cast<seconds>(std::chrono::steady_clock::now() - time_stamp_));
    }

private:
    bool debug;
    const std::string config_path_;

    std::chrono::steady_clock::time_point time_stamp_;
    std::unique_ptr<v1::identifier::Identifier> identifier_;
    std::unique_ptr<solver::Solver> pnp_solver_;
    std::shared_ptr<state_machine::StateMachine> state_machine_;
    std::shared_ptr<predictor::CarPredictorManager> live_target_manager_;
    std::unique_ptr<world_exe::v1::Syncer> syncer_;
    std::unique_ptr<fire_control::FireController> fire_controller_;
};

AutoAimSystem::AutoAimSystem(const bool& debug)
    : pimpl_(std::make_unique<Impl>(debug)) { }
AutoAimSystem::~AutoAimSystem() = default;
}
