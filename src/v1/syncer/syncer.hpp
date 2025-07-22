#pragma once

#include "data/rmcs_sync_data.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/sync_block.hpp"

namespace world_exe::sync {
class Syncer : public interfaces::ISyncBlock<interfaces::IPreDictorUpdatePackage> {
public:
    Syncer();
    ~Syncer();

    std::tuple<const interfaces::IPreDictorUpdatePackage&, bool> await(
        double t_second = 2) override;

    void SetCameraCaptureEndTimeStamp(const std::time_t& time);

    void SetMainData(const interfaces::IArmorInCamera& armor_in_camera);

    void LoadCallback(const data::SyncData& data) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}