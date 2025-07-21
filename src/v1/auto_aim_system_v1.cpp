#include "./auto_aim_system_v1.hpp"
#include "event_bus.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/identifier.hpp"
#include "interfaces/pnp_solver.hpp"
#include "interfaces/sync_block.hpp"
#include "interfaces/target_predictor.hpp"
#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <string>

/// 这玩意全生命周期活跃，直接分配然后丢一边，反正有回调
class world_exe::v1::SystemV1::SystemV1Impl {
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

    const std::string carmera_capture_end_time_stamp_event //
        = "/alliance_auto_aim/camera/cpature/end";

    const std::string carmera_capture_begin_time_stamp_event //
        = "/alliance_auto_aim/camera/cpature/begin";

    const std::string armors_in_camera_pnp_event //
        = "/alliance_auto_aim/armor_in_camera/pnp";

    const std::string car_tracing_event //
        = "/alliance_auto_aim/state/car_tracing";

    const std::string tracker_update_event //
        = "/alliance_auto_aim/tracker/update_package";

    const std::string get_lastest_predictor_ //
        = "/alliance_auto_aim/tracker/lastest_predictor";

    const std::string tracker_current_armors_event //
        = "/alliance_auto_aim/tracker/armors_detected_current";

    SystemV1Impl() {
        throw std::runtime_error("No Implement");

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
                    armors_in_image_identify_event, pnp_solver_->SolvePnp(data));
            });

        core::EventBus::Subscript<enumeration::CarIDFlag>(car_id_identify_event, //
            [this](const auto& data) {
                car_state_->Update(data);
                core::EventBus::Publish<enumeration::CarIDFlag>( //
                    armors_in_image_identify_event, car_state_->GetAllowdToFires());
            });
    }

private:
    interfaces::IIdentifier* identifier_;
    interfaces::IPnpSolver* pnp_solver_;
    interfaces::ICarState* car_state_;
    interfaces::ISyncBlock<interfaces::ITargetPredictor>* sync_;

    SystemV1Impl(const SystemV1Impl&) = delete;
    ~SystemV1Impl()                   = delete;
};

void world_exe::v1::SystemV1::Build() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1Impl();
}
world_exe::v1::SystemV1::SystemV1Impl* world_exe::v1::SystemV1::instance_ = nullptr;