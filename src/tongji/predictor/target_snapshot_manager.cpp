#include "target_snapshot_manager.hpp"

#include <memory>
#include <unordered_map>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/enum_tools.hpp"
#include "in_gimbal_control_armor.hpp"
#include "tongji/predictor/target_snapshot.hpp"

namespace world_exe::tongji::predictor {
class TargetSnapshotManager::Impl {
public:
    Impl(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>& snapshots)
        : id_(id) {
        for (const auto& [flag, target] : snapshots) {
            snapshots_.emplace(flag, TargetSnapshot(*target));
        }
    }

    const enumeration::ArmorIdFlag& GetId() const { return id_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std::time_t& time_stamp) const {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (const auto& [flag, snapshot] : snapshots_) {
            result[flag] = snapshot.GetArmorGimbalControlSpacings();
        }
        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

private:
    std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> snapshots_;
    enumeration::ArmorIdFlag id_;
};

TargetSnapshotManager::TargetSnapshotManager(const enumeration::ArmorIdFlag& id,
    const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> snapshots)
    : pimpl_(std::make_unique<Impl>(id, snapshots)) { }
TargetSnapshotManager::~TargetSnapshotManager() = default;

const enumeration ::ArmorIdFlag& TargetSnapshotManager::GetId() const { return pimpl_->GetId(); }

std ::shared_ptr<interfaces::IArmorInGimbalControl> TargetSnapshotManager::Predictor(
    const std ::time_t& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}
}