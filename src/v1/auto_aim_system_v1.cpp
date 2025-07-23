#include "./auto_aim_system_v1.hpp"
#include "data/fire_control.hpp"
#include "data/sync_data.hpp"
#include "enum/armor_id.hpp"
#include "event_bus.hpp"
#include "fire_controller/fire_controller.hpp"
#include "identifier/identifier.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/identifier.hpp"
#include "interfaces/pnp_solver.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/sync_block.hpp"
#include "interfaces/target_predictor.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include "predictor/predictor_manager.hpp"

#include "./auto_aim_system_v1.hpp"
#include "./state_machine/state_machine.hpp"
#include "./syncer/sync_data.hpp"
#include "./syncer/syncer.hpp"

#include <memory>
#include <opencv2/core/mat.hpp>

#include "eigen3/Eigen/Dense"
#include "v1/pnpsolver/armor_pnp_solver.hpp"

#include <cassert>
#include <chrono>
class world_exe::v1::SystemV1::SystemV1Impl { };
/// 这玩意全生命周期活跃，直接分配然后丢一边，反正有回调
template <int i> class SystemV1ImplTemp : public world_exe::v1::SystemV1::SystemV1Impl {
public:
    SystemV1ImplTemp() {
        using namespace world_exe;
        using namespace v1;
        using namespace parameters;

#define FLOW_IN (x, y)
#define FLOW_OUT (x, y)
        if constexpr (i == 1) {
#ifdef FLOW_IN
#undef FLOW_IN
#endif
#ifdef FLOW_OUT
#undef FLOW_OUT
#endif

#define FLOW_IN(x, y) std::cerr << #x << " flow in " << #y << std::endl;
#define FLOW_OUT(x, y) std::cerr << #x << " flow out " << #y << std::endl;
        }
        tracker_    = predictor;
        identifier_ = identifier;
        fire_control->SetTargetCarID(enumeration::CarIDFlag::Base);
        fire_control_ = fire_control;
        sync_         = sync;
        state_machine->SetSwitchFrameCount(4);
        car_state_  = state_machine;
        pnp_solver_ = armor_pnp;
        loader->BindBlock(*sync);

        assert(identifier_ != nullptr);
        assert(pnp_solver_ != nullptr);
        assert(car_state_ != nullptr);
        assert(tracker_ != nullptr);
        assert(fire_control_ != nullptr);
        assert(sync_ != nullptr);

        core::EventBus::Subscript<cv::Mat>(ParamsForSystemV1::raw_image_event, //
            [this](const auto& data) {
                const auto& [armors, flag] = identifier_->identify(data);
                FLOW_IN(ParamsForSystemV1::raw_image_event, cv::Mat)
                if (flag != enumeration::ArmorIdFlag::None)
                    core::EventBus::Publish<std::shared_ptr<interfaces::IArmorInImage>>( //
                        ParamsForSystemV1::armors_in_image_identify_event, armors);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    ParamsForSystemV1::car_id_identify_event, flag);
                FLOW_OUT(ParamsForSystemV1::raw_image_event, cv::Mat)
            });
        core::EventBus::Subscript<std::shared_ptr<interfaces::IArmorInImage>>(
            ParamsForSystemV1::armors_in_image_identify_event, //
            [this](const std::shared_ptr<interfaces::IArmorInImage>& data) {
                FLOW_IN(
                    ParamsForSystemV1::armors_in_image_identify_event, interfaces::IArmorInImage)
                core::EventBus::Publish<interfaces::IArmorInCamera>( //
                    ParamsForSystemV1::armors_in_camera_pnp_event, pnp_solver_->SolvePnp(data));
                FLOW_OUT(
                    ParamsForSystemV1::armors_in_image_identify_event, interfaces::IArmorInImage)
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(
            ParamsForSystemV1::car_id_identify_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::car_id_identify_event, enumeration::CarIDFlag)
                car_state_->Update(data);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    ParamsForSystemV1::car_tracing_event, car_state_->GetAllowdToFires());
                FLOW_OUT(ParamsForSystemV1::car_id_identify_event, enumeration::CarIDFlag)
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(ParamsForSystemV1::car_tracing_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::car_tracing_event, enumeration::CarIDFlag)
                fire_control->SetTargetCarID(data);
                core::EventBus::Publish<interfaces::IArmorInGimbalControl>(
                    ParamsForSystemV1::tracker_current_armors_event, tracker_->Predict(data, 0));

                FLOW_OUT(ParamsForSystemV1::car_tracing_event, enumeration::CarIDFlag)
            });
        core::EventBus::Subscript<interfaces::IArmorInCamera>(
            ParamsForSystemV1::armors_in_camera_pnp_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::armors_in_camera_pnp_event, interfaces::IArmorInCamera)
                sync->SetMainData(data);
                const auto& [predictor, flag] = sync_->await();
                if (flag)
                    core::EventBus::Publish<
                        std::shared_ptr<interfaces::IPreDictorUpdatePackage>>( //
                        ParamsForSystemV1::tracker_update_event, predictor);
                FLOW_OUT(ParamsForSystemV1::armors_in_camera_pnp_event, interfaces::IArmorInCamera)
            });
        core::EventBus::Subscript<data::CameraGimbalMuzzleSyncData>(
            ParamsForSystemV1::camera_capture_transforms, //
            [this](const auto& data) {
                FLOW_IN(
                    ParamsForSystemV1::camera_capture_transforms, data::CameraGimbalMuzzleSyncData)
                time_point_ = std::chrono::steady_clock::now();
                loader->Load(data);
                FLOW_OUT(
                    ParamsForSystemV1::camera_capture_transforms, data::CameraGimbalMuzzleSyncData)
            });
        core::EventBus::Subscript<std::shared_ptr<interfaces::IPreDictorUpdatePackage>>(
            ParamsForSystemV1::tracker_update_event, //
            [this](const std::shared_ptr<interfaces::IPreDictorUpdatePackage>& data) {
                FLOW_IN(
                    ParamsForSystemV1::tracker_update_event, interfaces::IPreDictorUpdatePackage)
                fire_control->SetTimeStamp(data->GetTimeStamped().GetTimeStamp());
                tracker_->Update(data);
                FLOW_OUT(
                    ParamsForSystemV1::tracker_update_event, interfaces::IPreDictorUpdatePackage)
            });
        core::EventBus::Subscript<std::shared_ptr<interfaces::IArmorInGimbalControl>>(
            ParamsForSystemV1::tracker_current_armors_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::tracker_current_armors_event,
                    interfaces::IArmorInGimbalControl)
                fire_control->SetArmorsInGimbalControl(data);
                core::EventBus::Publish<enumeration::CarIDFlag>(
                    ParamsForSystemV1::get_lastest_predictor_event,
                    fire_control_->GetAttackCarId());
                FLOW_OUT(ParamsForSystemV1::tracker_current_armors_event,
                    interfaces::IArmorInGimbalControl)
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(
            ParamsForSystemV1::get_lastest_predictor_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::get_lastest_predictor_event, enumeration::CarIDFlag)
                if (data != enumeration::CarIDFlag::None)
                    core::EventBus::Publish<interfaces::IPredictor>(
                        ParamsForSystemV1::get_lastest_predictor_event,
                        tracker_->GetPredictor(data));
                FLOW_OUT(ParamsForSystemV1::get_lastest_predictor_event, enumeration::CarIDFlag)
            });
        core::EventBus::Subscript<std::shared_ptr<interfaces::IPredictor>>(
            ParamsForSystemV1::get_lastest_predictor_event, //
            [this](const auto& data) {
                FLOW_IN(ParamsForSystemV1::get_lastest_predictor_event, interfaces::IPredictor)
                fire_control->SetPredictor(data);
                core::EventBus::Publish<data::FireControl>(ParamsForSystemV1::fire_control_event,
                    fire_control_->CalculateTarget(
                        (std::chrono::steady_clock::now() - time_point_).count()));
                FLOW_OUT(ParamsForSystemV1::get_lastest_predictor_event, interfaces::IPredictor)
            });
    }

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time_point_ = {};
    std::shared_ptr<world_exe::interfaces::IIdentifier> identifier_;
    std::shared_ptr<world_exe::interfaces::IPnpSolver> pnp_solver_;
    std::shared_ptr<world_exe::interfaces::ICarState> car_state_;
    std::shared_ptr<world_exe::interfaces::ITargetPredictor> tracker_;
    std::shared_ptr<world_exe::interfaces::IFireControl> fire_control_;
    std::shared_ptr<
        world_exe::interfaces::ISyncBlock<world_exe::interfaces::IPreDictorUpdatePackage>>
        sync_;

private:
    std::shared_ptr<world_exe::v1::predictor::PredictorManager> predictor =
        std::make_shared<world_exe::v1::predictor::PredictorManager>();
    std::shared_ptr<world_exe::v1::identifier::Identifier> identifier =
        std::make_shared<world_exe::v1::identifier::Identifier>(
            world_exe::parameters::ParamsForSystemV1::szu_model_path(),
            world_exe::parameters::ParamsForSystemV1::device(),
            world_exe::parameters::HikCameraProfile::get_width(),
            world_exe::parameters::HikCameraProfile::get_height());
    std::shared_ptr<world_exe::v1::fire_control::TracingFireControl> fire_control =
        std::make_shared<world_exe::v1::fire_control::TracingFireControl>(
            world_exe::parameters::ParamsForSystemV1::control_delay_in_second(),
            world_exe::parameters::ParamsForSystemV1::velocity_begin(),
            world_exe::parameters::ParamsForSystemV1::gravity());
    std::shared_ptr<world_exe::v1::sync::Syncer> sync =
        std::make_shared<world_exe::v1::sync::Syncer>();
    std::shared_ptr<world_exe::v1::sync::SyncLoad> loader =
        std::make_shared<world_exe::v1::sync::SyncLoad>();
    std::shared_ptr<world_exe::v1::state_machine::StateMachine> state_machine =
        std::make_shared<world_exe::v1::state_machine::StateMachine>();
    std::shared_ptr<world_exe::v1::pnpsolver::ArmorIPPEPnPSolver> armor_pnp =
        std::make_shared<world_exe::v1::pnpsolver::ArmorIPPEPnPSolver>();
    SystemV1ImplTemp(const SystemV1ImplTemp&) = delete;
    ~SystemV1ImplTemp()                       = delete;
};

void world_exe::v1::SystemV1::Build() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1ImplTemp<0>();
}
void world_exe::v1::SystemV1::Build_1() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1ImplTemp<1>();
}
world_exe::v1::SystemV1::SystemV1Impl* world_exe::v1::SystemV1::instance_ = nullptr;