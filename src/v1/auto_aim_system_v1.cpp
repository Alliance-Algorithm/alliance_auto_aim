#include "./auto_aim_system_v1.hpp"
#include "data/fire_control.hpp"
#include "data/sync_data.hpp"
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
#include "parameters.hpp"
#include "predictor/predictor_manager.hpp"

#include "./auto_aim_system_v1.hpp"
#include "./state_machine/state_machine.hpp"
#include "./syncer/sync_data.hpp"
#include "./syncer/syncer.hpp"

#include <opencv2/core/mat.hpp>

#include "eigen3/Eigen/Dense"

#include <cassert>
#include <chrono>
#include <stdexcept>

/// 这玩意全生命周期活跃，直接分配然后丢一边，反正有回调
class world_exe::v1::SystemV1::SystemV1Impl {
public:
    SystemV1Impl() {
        using namespace world_exe;
        using namespace v1;
        using namespace v1::parameters;

        predictor::PredictorManager* predictor = new predictor::PredictorManager();
        tracker_                               = predictor;
        identifier::Identifier* identifier     = new identifier::Identifier(model_path, device);
        identifier_                            = identifier;
        fire_control::TracingFireControl* fire_control =
            new fire_control::TracingFireControl(control_delay_in_second, velocity_begin, gravity);
        fire_control_          = fire_control;
        sync::Syncer* sync     = new sync::Syncer();
        sync_                  = sync;
        sync::SyncLoad* loader = new sync::SyncLoad();
        loader->BindBlock(*sync);
        state_machine::StateMachine* state_machine = new state_machine::StateMachine();
        state_machine->SetSwitchFrameCount(4);
        car_state_ = state_machine;

        assert(identifier_ != nullptr);
        assert(pnp_solver_ != nullptr);
        assert(car_state_ != nullptr);
        assert(tracker_ != nullptr);
        assert(fire_control_ != nullptr);
        assert(sync_ != nullptr);

        core::EventBus::Subscript<cv::Mat>(raw_image_event, //
            [this](const auto& data) {
                const auto& [armors, flag] = identifier_->identify(data);
                core::EventBus::Publish<interfaces::IArmorInImage>( //
                    armors_in_image_identify_event, armors);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    car_id_identify_event, flag);
            });
        core::EventBus::Subscript<interfaces::IArmorInImage>(armors_in_image_identify_event, //
            [this](const auto& data) {
                core::EventBus::Publish<interfaces::IArmorInCamera>( //
                    armors_in_camera_pnp_event, pnp_solver_->SolvePnp(data));
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(car_id_identify_event, //
            [this](const auto& data) {
                car_state_->Update(data);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    car_tracing_event, car_state_->GetAllowdToFires());
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(car_tracing_event, //
            [this, &fire_control](const auto& data) {
                fire_control->SetTargetCarID(data);
                core::EventBus::Publish<interfaces::IArmorInGimbalControl>(
                    tracker_current_armors_event, tracker_->Predict(data, 0));
            });
        core::EventBus::Subscript<interfaces::IArmorInCamera>(armors_in_camera_pnp_event, //
            [this](const auto& data) {
                const auto& [predictor, flag] = sync_->await();
                if (flag)
                    core::EventBus::Publish<interfaces::IPreDictorUpdatePackage>( //
                        tracker_update_event, predictor);
            });
        core::EventBus::Subscript<data::CameraGimbalMuzzleSyncData>(camera_capture_transforms, //
            [this, &loader](const auto& data) {
                time_point_ = std::chrono::steady_clock::now();
                loader->Load(data);
            });
        core::EventBus::Subscript<interfaces::IPreDictorUpdatePackage>(tracker_update_event, //
            [this, &fire_control](const interfaces::IPreDictorUpdatePackage& data) {
                fire_control->SetTimeStamp(data.GetTimeStamped().GetTimeStamp());
                tracker_->Update(data);
            });
        core::EventBus::Subscript<interfaces::IArmorInGimbalControl>(
            tracker_current_armors_event, //
            [this, &fire_control](const auto& data) {
                fire_control->SetArmorsInGimbalControl(data);
                core::EventBus::Publish<enumeration::CarIDFlag>(
                    get_lastest_predictor_event, fire_control_->GetAttackCarId());
            });
        core::EventBus::Subscript<enumeration::CarIDFlag>(get_lastest_predictor_event, //
            [this](const auto& data) {
                core::EventBus::Publish<interfaces::IPredictor>(
                    get_lastest_predictor_event, tracker_->GetPredictor(data));
            });
        core::EventBus::Subscript<interfaces::IPredictor>(get_lastest_predictor_event, //
            [this, &fire_control](const auto& data) {
                fire_control->SetPredictor(data);
                core::EventBus::Publish<data::FireControl>(get_lastest_predictor_event,
                    fire_control_->CalculateTarget(
                        (std::chrono::steady_clock::now() - time_point_).count()));
            });
    }

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time_point_ = {};
    world_exe::interfaces::IIdentifier* identifier_;
    world_exe::interfaces::IPnpSolver* pnp_solver_;
    world_exe::interfaces::ICarState* car_state_;
    world_exe::interfaces::ITargetPredictor* tracker_;
    world_exe::interfaces::IFireControl* fire_control_;
    world_exe::interfaces::ISyncBlock<world_exe::interfaces::IPreDictorUpdatePackage>* sync_;

private:
    SystemV1Impl(const SystemV1Impl&) = delete;
    ~SystemV1Impl()                   = delete;
};

void world_exe::v1::SystemV1::Build() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1Impl();
}
world_exe::v1::SystemV1::SystemV1Impl* world_exe::v1::SystemV1::instance_ = nullptr;