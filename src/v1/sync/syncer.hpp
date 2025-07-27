#pragma once

#include "data/sync_data.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"
#include <interfaces/sync_block.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::v1 {
class Syncer final
    : public world_exe::interfaces::ISyncBlock<world_exe::interfaces::IPreDictorUpdatePackage> {
public:
    Syncer();
    ~Syncer();

    void set_armor_pnp(const std::shared_ptr<interfaces::IArmorInCamera>& armor_pnp);
    void set_camera_sync_data(const data::CameraGimbalMuzzleSyncData& camera_data);

    std::tuple<std::shared_ptr<world_exe::interfaces::IPreDictorUpdatePackage>, bool> await(
        double t_second) override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    std::shared_ptr<interfaces::IPreDictorUpdatePackage> last_;
};
}