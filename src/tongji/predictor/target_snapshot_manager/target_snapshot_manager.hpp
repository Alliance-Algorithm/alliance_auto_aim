#pragma once

#include <memory>
#include <unordered_map>

#include "../live_target_manager/live_target.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::tongji::predictor {
class TargetSnapshotManager final : public interfaces::IPredictor {
public:
    TargetSnapshotManager(const std::string& config_path,
        const std::unordered_map<enumeration::ArmorIdFlag, std::unique_ptr<LiveTarget>>&
            live_target_map,
        const data::TimeStamp& time_stamp);
    ~TargetSnapshotManager();

    const enumeration ::ArmorIdFlag& GetId() const override;
    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const override;

    TargetSnapshotManager(const TargetSnapshotManager&)                = delete;
    TargetSnapshotManager& operator=(const TargetSnapshotManager&)     = delete;
    TargetSnapshotManager(TargetSnapshotManager&&) noexcept            = default;
    TargetSnapshotManager& operator=(TargetSnapshotManager&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
