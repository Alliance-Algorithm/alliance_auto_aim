#pragma once

#include <memory>
#include <unordered_map>

#include "../live_target_manager/live_target.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::tongji::predictor {
class TargetSnapshotManagerImpl;
struct GimbalCommand {
    double yaw;
    double pitch;
};

class TargetSnapshotManager final : public interfaces::IPredictor {
public:
    TargetSnapshotManager(const std::string& config_path, const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
            live_target_map,
        const std::time_t& now, const double& bullet_speed);
    ~TargetSnapshotManager();

    const enumeration ::ArmorIdFlag& GetId() const override;
    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

    auto GetGimbalCommand() const -> GimbalCommand const;

    TargetSnapshotManager(const TargetSnapshotManager&)                = delete;
    TargetSnapshotManager& operator=(const TargetSnapshotManager&)     = delete;
    TargetSnapshotManager(TargetSnapshotManager&&) noexcept            = default;
    TargetSnapshotManager& operator=(TargetSnapshotManager&&) noexcept = default;

private:
    std::unique_ptr<TargetSnapshotManagerImpl> pimpl_;
};
}
