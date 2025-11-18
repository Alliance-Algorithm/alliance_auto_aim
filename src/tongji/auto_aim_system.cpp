#include "auto_aim_system.hpp"

#include <chrono>

#include <cstdio>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
// #include <print>
#include <ostream>
#include <tuple>

#include "../v1/sync/syncer.hpp"
#include "core/event_bus.hpp"
#include "data/mat_stamped.hpp"
#include "data/predictor_update_package.hpp"
#include "data/sync_data.hpp"
#include "data/time_stamped.hpp"
#include "enum/car_id.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include "tongji/fire_controller/fire_controller.hpp"
#include "tongji/predictor/car_predictor/car_predictor_manager.hpp"
#include "tongji/solver/solver.hpp"
#include "tongji/state_machine/state_machine.hpp"
#include "utils/fps_counter.hpp"
// #include "utils/visualization.hpp"
#include "v1/identifier/identifier.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace world_exe::tongji {
using namespace std::chrono;

class AutoAimSystem::Impl {
public:
    explicit Impl(const bool& debug)
        : debug(debug)

        , fps_() {

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("allianc"
                                                                                           "e_"
                                                                                           "ros_"
                                                                                           "auto_"
                                                                                           "aim");

        std::filesystem::path config_fs_path =
            std::filesystem::path(package_share_directory) / "example.yaml";
        std::string model_path = config_fs_path.string();

        std::filesystem::path model_fs_path =
            std::filesystem::path(package_share_directory) / "szu_identify_model.onnx";
        config_path_ = config_fs_path.string();

        identifier_ = std::make_unique<v1::identifier::Identifier>(
            parameters::ParamsForSystemV1::szu_model_path(),
            parameters::ParamsForSystemV1::device(), parameters::HikCameraProfile::get_width(),
            parameters::HikCameraProfile::get_height());
        // identifier_          = std::make_unique<tongji::identifier::Identifier>(config_path_,
        // ".");
        pnp_solver_          = std::make_unique<solver::Solver>();
        live_target_manager_ = std::make_shared<predictor::CarPredictorManager>(config_path_);
        state_machine_       = std::make_shared<state_machine::StateMachine>();
        fire_controller_     = std::make_unique<fire_control::FireController>(
            config_path_, state_machine_, live_target_manager_);
        syncer_ = std::make_unique<world_exe::v1::Syncer>(seconds(2), 6e-6);

        core::EventBus::Subscript<world_exe::data::MatStamped>(
            parameters::ParamsForSystemV1::raw_image_event,
            [this](const world_exe::data::MatStamped& mat) { Solve(mat); });
        core::EventBus::Subscript<data::CameraGimbalMuzzleSyncData>(
            parameters::ParamsForSystemV1::camera_capture_transforms,
            [this](const auto& pkg) { SetTransfroms(pkg); });
    }

    auto Solve(const data::MatStamped& raw) -> void {
        if (identifier_ == nullptr) std::terminate();
        const auto& [armors_in_image, flag] = identifier_->identify(raw.mat);

        // if (armors_in_image) {
        //     auto visualized = raw.mat.clone();
        //     util::visualization::draw_armor_in_image(*armors_in_image, visualized);
        //     cv::imshow("identified", visualized);
        //     cv::waitKey(1);
        // }
        // if (fps_.count()) std::cout << fps_.fps() << std::endl;

        if (flag == enumeration::ArmorIdFlag::None) {
            state_machine_->SetLostState();
            // std::cout << "no armor identified!" << std::endl;
            return;
        } else {
            // std::cout << "found!" << std::endl;
        }

        // TODO:update invincible_armors
        state_machine_->Update(armors_in_image, enumeration::CarIDFlag::None, time_stamp_);

        std::cout << "img stamp:" << raw.stamp.to_nanosec() << std::endl;
        // 这里使用 any_clock::now 也可以，但是时间系统的转换和同步我希望是单独的部分
        auto [pack, check] = syncer_->get_data(raw.stamp);
        if (!check) {
            // TODO：等待传入真实数据
            // pack.camera_capture_begin_time_stamp =
            //     data::TimeStamp(steady_clock::now().time_since_epoch());
            std::cout << " no sync data" << std::endl;
            return;
        }

        pnp_solver_->SetCamera2Gimbal(pack.camera_to_gimbal);
        const auto& armors_in_camera = pnp_solver_->SolvePnp(armors_in_image);

        auto combined = std::make_shared<data::PredictorUpdatePackage>(pack, armors_in_camera);

        live_target_manager_->Update(combined);

        core::EventBus::Publish<std ::shared_ptr<interfaces ::IArmorInGimbalControl>>(
            parameters::ParamsForSystemV1::tracker_current_armors_event,
            live_target_manager_->Predict(flag, pack.camera_capture_begin_time_stamp));

        time_stamp_ = pack.camera_capture_begin_time_stamp;

        /// 这里应该有一个线程进行稳定的输出之类的
        /// 轨迹规划器没有实现，先不管

        fire_controller_->SetGimbal2Muzzle(pack.gimbal_to_muzzle);
        core::EventBus::Publish<data::FireControl>(
            parameters::ParamsForSystemV1::fire_control_event, GetControlCommand());

        if (!debug) [[likely]]
            return;

        auto target = state_machine_->GetAllowdToFires();
        // std::cout << "target:" << util::stringifier::ToString(target) << std::endl;

        core::EventBus::Publish<enumeration::CarIDFlag>(
            parameters::ParamsForSystemV1::car_id_identify_event, flag);
        core::EventBus::Publish<std::shared_ptr<interfaces::IArmorInImage>>(
            parameters::ParamsForSystemV1::armors_in_image_identify_event, armors_in_image);
        core::EventBus::Publish<std::shared_ptr<world_exe::interfaces::IArmorInCamera>>(
            parameters::ParamsForSystemV1::armors_in_camera_pnp_event, armors_in_camera);
        core::EventBus::Publish<std::shared_ptr<data::PredictorUpdatePackage>>(
            parameters::ParamsForSystemV1::tracker_update_event, combined);
        core::EventBus::Publish<enumeration::CarIDFlag>(
            parameters::ParamsForSystemV1::car_tracing_event, state_machine_->GetAllowdToFires());
        core::EventBus::Publish<std ::shared_ptr<interfaces ::IArmorInGimbalControl>>(
            parameters::ParamsForSystemV1::get_lastest_predictor_event,
            fire_controller_->GetArmorsSnapshot());
        // if (armors_in_image) {
        //     auto visualized = raw.mat.clone();
        //     util::visualization::draw_armor_in_image(*armors_in_image, visualized);
        //     cv::imshow("identified", visualized);
        //     cv::waitKey(1);
        // } else {
        //     std::printf("No identified armors/n");
        // }
        // if (armors_in_camera) {
        //     auto visualized = raw.mat.clone();
        //     util::visualization::draw_armor_in_camera(*armors_in_camera,
        //         parameters::HikCameraProfile::get_intrinsic_parameters(),
        //         parameters::HikCameraProfile::get_distortion_parameters(),
        //         parameters::Robomaster::NormalArmorObjectPointsRos, visualized);
        //     cv::imshow("pnp", visualized);
        //     cv::waitKey(1);
        // } else {
        //     std::printf("No pnp armors/n");
        // }
    }

    void SetTransfroms(const data::CameraGimbalMuzzleSyncData& data) { syncer_->set_data(data); }

    // TODO:时间戳有待fix
    data::FireControl GetControlCommand() {
        auto attacked_id = fire_controller_->GetAttackCarId();
        return fire_controller_->CalculateTarget(
            data::TimeStamp(steady_clock::now().time_since_epoch()));
    }

private:
    bool debug;
    std::string config_path_;
    world_exe::util::FpsCounter fps_;
    data::TimeStamp time_stamp_;
    std::unique_ptr<world_exe::v1::identifier::Identifier> identifier_;
    std::unique_ptr<solver::Solver> pnp_solver_;
    std::shared_ptr<state_machine::StateMachine> state_machine_;
    std::shared_ptr<predictor::CarPredictorManager> live_target_manager_;
    std::unique_ptr<world_exe::v1::Syncer> syncer_;
    std::unique_ptr<fire_control::FireController> fire_controller_;
};

AutoAimSystem::AutoAimSystem(const bool& debug)
    : pimpl_(std::make_unique<Impl>(debug)) { }
AutoAimSystem::~AutoAimSystem() = default;

std::unique_ptr<AutoAimSystem> AutoAimSystem::v2;
void AutoAimSystem::build(bool debug) {
    if (v2 != nullptr) return;
    v2 = std::make_unique<AutoAimSystem>(debug);
}
}
