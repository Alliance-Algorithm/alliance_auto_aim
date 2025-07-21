#include "./auto_aim_system_v1.hpp"
#include "concepts/pnp_solver.hpp"
#include "data/fire_control.hpp"
#include "event_bus.hpp"
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
#include "predictor/predictor_manager.hpp"
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <stdexcept>

class world_exe::v1::SystemV1::SystemV1ImplBase { };

/// 这玩意全生命周期活跃，直接分配然后丢一边，反正有回调
template <world_exe::concepts::concept_pnp_solver TPnpSolver>
class SystemV1Impl : public world_exe::v1::SystemV1::SystemV1ImplBase {
public:
    const std::string raw_image_event //
        = "/alliance_auto_aim/cv_mat/raw";

    const std::string armors_in_image_identify_event //
        = "/alliance_auto_aim/armor_in_image/identified";

    const std::string car_id_identify_event //
        = "/alliance_auto_aim/car_id_flag/identified";

    const std::string camera_to_gimbal_control_spacing_event //
        = "/alliance_auto_aim/transfrom/camera_to_gimbal_control";

    const std::string gimbal_control_to_muzzle_event //
        = "/alliance_auto_aim/transfrom/gimbal_control_to_muzzle";

    const std::string camera_capture_end_time_stamp_event //
        = "/alliance_auto_aim/camera/cpature/end";

    const std::string camera_capture_begin_time_stamp_event //
        = "/alliance_auto_aim/camera/cpature/begin";

    const std::string armors_in_camera_pnp_event //
        = "/alliance_auto_aim/armor_in_camera/pnp";

    const std::string car_tracing_event //
        = "/alliance_auto_aim/state/car_tracing";

    const std::string tracker_update_event //
        = "/alliance_auto_aim/tracker/update_package";

    const std::string get_lastest_predictor_event //
        = "/alliance_auto_aim/tracker/lastest_predictor";

    const std::string tracker_current_armors_event //
        = "/alliance_auto_aim/tracker/armors_detected_current";

    const std::string model_path = "/alliance_auto_aim/tracker/armors_detected_current";

    SystemV1Impl(TPnpSolver& pnp_solver)
        : pnp_solver_(pnp_solver) {
        throw std::runtime_error("No Implement");
        using namespace world_exe;

        predictor::PredictorManager* predictor = new predictor::PredictorManager();
        tracker_                               = *predictor;
        identifier::Identifier* identifier     = new identifier::Identifier(model_path, "AUTO");
        identifier_                            = *identifier;

        core::EventBus::Subscript<cv::Mat>(raw_image_event, //
            [this](const auto& data) {
                const auto& [armors, flag] = identifier_.identify(data);
                core::EventBus::Publish<interfaces::IArmorInImage>( //
                    armors_in_image_identify_event, armors);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    car_id_identify_event, flag);
            });

        core::EventBus::Subscript<interfaces::IArmorInImage>(armors_in_image_identify_event, //
            [this](const auto& data) {
                core::EventBus::Publish<interfaces::IArmorInCamera>( //
                    armors_in_camera_pnp_event, pnp_solver_.SolvePnp(data));
            });

        core::EventBus::Subscript<enumeration::CarIDFlag>(car_id_identify_event, //
            [this](const auto& data) {
                car_state_.Update(data);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    car_tracing_event, car_state_.GetAllowdToFires());
            });

        core::EventBus::Subscript<enumeration::CarIDFlag>(car_tracing_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set fire_controller target here");
                core::EventBus::Subscript<interfaces::IArmorInGimbalControl>(
                    tracker_current_armors_event, tracker_.Predict(data, 0));
            });

        core::EventBus::Subscript<interfaces::IArmorInCamera>(armors_in_camera_pnp_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set sync component armor_in_camera here");
                const auto& [predictor, flag] = sync_.await();
                if (flag)
                    core::EventBus::Publish<interfaces::IPreDictorUpdatePackage>( //
                        tracker_update_event, predictor);
            });

        core::EventBus::Subscript<std::time_t>(camera_capture_begin_time_stamp_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set sync component camera_capture_begin here");
                throw std::runtime_error("you should reset fire control time here here");
                time_point_ = std::chrono::steady_clock::now();
            });

        core::EventBus::Subscript<Eigen::Affine3d>(camera_to_gimbal_control_spacing_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set sync component camera_to_gimbal here");
            });

        core::EventBus::Subscript<std::time_t>(camera_capture_end_time_stamp_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set sync component camera_capture_end "
                                         "here");
            });

        core::EventBus::Subscript<Eigen::Affine3d>(camera_to_gimbal_control_spacing_event, //
            [this](const auto& data) {
                throw std::runtime_error("you should set sync component camera_capture_end "
                                         "here");
            });

        core::EventBus::Subscript<interfaces::IPreDictorUpdatePackage>(tracker_update_event, //
            [this](const auto& data) { tracker_.Update(data); });

        core::EventBus::Subscript<interfaces::IArmorInGimbalControl>(
            tracker_current_armors_event, //
            [this](const auto& data) {
                throw std::runtime_error("fire control calculate target here");
                core::EventBus::Publish<enumeration::CarIDFlag>(
                    get_lastest_predictor_event, fire_control_.GetAttackCarId());
            });

        core::EventBus::Subscript<enumeration::CarIDFlag>(get_lastest_predictor_event, //
            [this](const auto& data) {
                core::EventBus::Publish<interfaces::IPredictor>(
                    get_lastest_predictor_event, tracker_.GetPredictor(data));
            });
        core::EventBus::Subscript<interfaces::IPredictor>(get_lastest_predictor_event, //
            [this](const auto& data) {
                throw std::runtime_error("fire control set predictor here");
                core::EventBus::Publish<data::FireControl>(get_lastest_predictor_event,
                    fire_control_.CalculateTarget(
                        (std::chrono::steady_clock::now() - time_point_).count()));
            });
    }

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time_point_ = {};
    world_exe::interfaces::IIdentifier& identifier_;
    world_exe::interfaces::IPnpSolver& pnp_solver_;
    world_exe::interfaces::ICarState& car_state_;
    world_exe::interfaces::ITargetPredictor& tracker_;
    world_exe::interfaces::IFireControl& fire_control_;
    world_exe::interfaces::ISyncBlock<world_exe::interfaces::IPreDictorUpdatePackage>& sync_;

private:
    SystemV1Impl(const SystemV1Impl&) = delete;
    ~SystemV1Impl()                   = delete;
};

void world_exe::v1::SystemV1::Build() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1::SystemV1ImplBase;
}
world_exe::v1::SystemV1::SystemV1ImplBase* world_exe::v1::SystemV1::instance_ = nullptr;