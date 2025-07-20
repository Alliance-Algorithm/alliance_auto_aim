#include "./auto_aim_system_v1.hpp"
#include "event_bus.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/identifier.hpp"
#include "interfaces/pnp_solver.hpp"
#include "interfaces/sync_block.hpp"
#include "interfaces/target_predictor.hpp"
#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <string>

using namespace std;
/// 这玩意全生命周期活跃，直接分配然后丢一边，反正有回调
class world_exe::v1::SystemV1::SystemV1Impl {
public:
    const string RawImageEvent           = "/alliance_auto_aim/cv_mat/raw";
    const string ArmorIdentifyEvent      = "/alliance_auto_aim/armor_in_image/identified";
    const string CarIDIdentifyEvent      = "/alliance_auto_aim/car_id_flag/identified";
    const string IPreDictorUpdatePackage = "/alliance_auto_aim/armor_in_camera/pnp";

    SystemV1Impl() {
        throw new std::runtime_error("No Implement");

        core::EventBus::Subscript<cv::Mat>(RawImageEvent,

            [this](const cv::Mat& mat) {
                const auto& [armors, ids] = identifier_->identify(mat);
                core::EventBus::Publish(ArmorIdentifyEvent, armors);
                core::EventBus::Publish<enumeration::CarIDFlag>(CarIDIdentifyEvent, ids);
            });
        //

        core::EventBus::Subscript<interfaces::IArmorInImage>(ArmorIdentifyEvent,

            [this](const interfaces::IArmorInImage& armor2d) {
                core::EventBus::Publish<interfaces::IArmorInCamera>(
                    ArmorIdentifyEvent, pnp_solver_->SolvePnp(armor2d));
            });
        //

        core::EventBus::Subscript<enumeration::CarIDFlag>(CarIDIdentifyEvent,

            [this](const enumeration::CarIDFlag& id_detected) {
                car_state_->Update(id_detected);

                core::EventBus::Publish<enumeration::CarIDFlag>(
                    ArmorIdentifyEvent, car_state_->GetAllowdToFires());
            });
        //

        core::EventBus::Subscript<interfaces::IArmorInCamera>(
            ArmorIdentifyEvent, [this](const interfaces::IArmorInCamera& armor2d) {
                const auto& [data, flag] = sync_->await(0.5);
                if (flag)
                    core::EventBus::Publish<interfaces::ITargetPredictor>(ArmorIdentifyEvent, data);
            });
        //
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