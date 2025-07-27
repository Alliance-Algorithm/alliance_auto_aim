
#include "v1/sync/syncer.hpp"
#include "data/sync_data.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/time_stamped.hpp"
#include <chrono>
#include <interfaces/sync_block.hpp>
#include <memory>

namespace world_exe::v1 {

struct Syncer::Impl final : public interfaces::IPreDictorUpdatePackage,
                            public interfaces::ITimeStamped,
                            public interfaces::IArmorInCamera {

    const ITimeStamped& GetTimeStamped() const override { return *this; }

    const std::time_t& GetTimeStamp() const override { return last_update_time_; }

    std::shared_ptr<interfaces::IArmorInCamera> GetArmors() const override {
        if (armors_loaded_ == nullptr) return std::make_shared<Impl>();
        return armors_loaded_;
    }

    /// Affine form image to gimbal_control
    Eigen::Affine3d GetTransform() const final { return transform_loaded_; }

    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        const static std::vector<data::ArmorCameraSpacing> empty {};
        return empty;
    };

    std::shared_ptr<interfaces::IPreDictorUpdatePackage> await(std::time_t time) {
        auto begin = std::chrono::steady_clock::now();
        while ((std::chrono::steady_clock::now() - begin).count() < time) {
            if (armors_loaded_ == nullptr) continue;
            if (camera_sync_data_.camera_capture_begin_time_stamp == last_update_time_) continue;

            return std::make_shared<Impl>(armors_loaded_, camera_sync_data_);
        }
        return nullptr;
    }

    std::shared_ptr<interfaces::IArmorInCamera> armors_loaded_ = nullptr;
    Eigen::Affine3d transform_loaded_                          = {};
    data::CameraGimbalMuzzleSyncData camera_sync_data_         = {};
    std::time_t last_update_time_                              = 0;

    Impl()  = default;
    ~Impl() = default;
    Impl(const std::shared_ptr<interfaces::IArmorInCamera>& armors_loaded,
        const data::CameraGimbalMuzzleSyncData& camera_sync_data_)
        : armors_loaded_(armors_loaded)
        , transform_loaded_(camera_sync_data_.camera_to_gimbal) { }

private:
};

Syncer::Syncer()
    : pimpl_(std::make_unique<Impl>()) { }

Syncer::~Syncer() = default;

void Syncer::set_armor_pnp(const std::shared_ptr<interfaces::IArmorInCamera>& armor_pnp) {
    pimpl_->armors_loaded_ = armor_pnp;
}
void Syncer::set_camera_sync_data(const data::CameraGimbalMuzzleSyncData& camera_data) {
    pimpl_->camera_sync_data_ = camera_data;
}

std::tuple<std::shared_ptr<interfaces::IPreDictorUpdatePackage>, bool> Syncer::await(
    double t_second) {
    auto ret = pimpl_->await(static_cast<time_t>(t_second * 1e9));
    if (ret != nullptr) last_ = ret;

    return { last_, ret != nullptr };
}

}