#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/sync_block.hpp"

namespace world_exe::sync {
struct SyncData {
    std::time_t camera_capture_begin_time_stamp;
    Eigen::Affine3d camera_to_gimbal;
    Eigen::Affine3d gimbal_to_muzzle;
};
class Syncer : public interfaces::ISyncBlock<interfaces::IPreDictorUpdatePackage> {
public:
    Syncer();
    ~Syncer();

    std::tuple<const interfaces::IPreDictorUpdatePackage&, bool> await(
        double t_second = 2) override;

    void setTimeStamp(const std::time_t& time);

    void SetMainData(const interfaces::IArmorInCamera& armor_in_camera);

    void LoadCallback(const SyncData& data) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}