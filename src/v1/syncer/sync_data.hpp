#pragma once

#include "interfaces/sync_load.hpp"
#include "syncer.hpp"

namespace world_exe::interfaces {
template <> class ISyncLoad<sync::SyncData> {
public:
    void Load(const sync::SyncData& data) {
        sync_data_ = data;
        load_callback_();
    };

    void BindBlock(const sync::Syncer& syncer) {
        load_callback_ = [&]() { syncer.LoadCallback(sync_data_); };
    }

private:
    std::function<void()> load_callback_;
    sync::SyncData sync_data_;
};
}