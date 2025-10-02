#pragma once

#include <memory>
#include <unordered_map>

#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"
#include "tongji/predictor/live_target.hpp"
#include "tongji/predictor/target_snapshot.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshotManager final : public interfaces::IPredictor {
public:
    TargetSnapshotManager(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> snapshots);
    ~TargetSnapshotManager();

    const enumeration ::ArmorIdFlag& GetId() const override;

    const std::unique_ptr<TargetSnapshot> GetSingleSnapshot(
        const enumeration::ArmorIdFlag& id) const;

    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
