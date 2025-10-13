#pragma once

#include <memory>
#include <unordered_map>

#include "../live_target_manager/live_target.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::tongji::predictor {

struct GimbalCommand {
    double yaw;
    double pitch;
};

class TargetSnapshotManager final : public interfaces::IPredictor {
public:
    TargetSnapshotManager(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
            live_target_map,
        const std::time_t& now, const double& bullet_speed, const double& yaw_offset,
        const double& pitch_offset);
    ~TargetSnapshotManager();

    const enumeration ::ArmorIdFlag& GetId() const override;
    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

    auto GetGimbalCommand() const -> GimbalCommand const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
