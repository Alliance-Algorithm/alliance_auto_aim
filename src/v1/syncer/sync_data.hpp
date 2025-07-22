#pragma once

#include "data/sync_data.hpp"
#include "interfaces/sync_load.hpp"
#include "syncer.hpp"
namespace world_exe::sync {
class SyncLoad : public world_exe::interfaces::ISyncLoad<data::CameraGimbalMuzzleSyncData> {
public:
    void Load(const data::CameraGimbalMuzzleSyncData& data) override {
        sync_data_ = data;
        load_callback_();
    };

    void BindBlock(const Syncer& syncer) {
        load_callback_ = [&]() { syncer.LoadCallback(sync_data_); };
    }

private:
    std::function<void()> load_callback_;
    data::CameraGimbalMuzzleSyncData sync_data_;
};
}