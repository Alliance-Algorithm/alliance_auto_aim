#include "target_snapshot_manager.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "../in_gimbal_control_armor.hpp"
#include "../live_target_manager/live_target.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/enum_tools.hpp"
#include "target_snapshot.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshotManager::Impl {
public:
    Impl(const std::string& config_path,
        const std::unordered_map<enumeration::ArmorIdFlag, std::unique_ptr<LiveTarget>>&
            live_target_map,
        const data::TimeStamp& time_stamp)
        : snapshots_(BuildSnapshots(live_target_map))
        , time_stamp_(time_stamp) {
        for (const auto& [id, live_target] : live_target_map) {
            ids_ = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(id) | static_cast<uint32_t>(ids_));
        }
    }

    const enumeration::ArmorIdFlag& GetId() const { return ids_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const {

        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (const auto& [id, snapshot] : snapshots_) {
            result.emplace(id,
                snapshot.GetPredictedArmorGimbalControlSpacings(
                    (time_stamp - time_stamp_).to_seconds()));
        }
        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

private:
    static std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> BuildSnapshots(
        const std::unordered_map<enumeration::ArmorIdFlag, std::unique_ptr<LiveTarget>>& input) {
        std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> result;
        for (const auto& [id, target] : input) {
            if (target) {
                result.emplace(id, TargetSnapshot(*target));
            }
        }
        return result;
    }

    const std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> snapshots_;
    const data::TimeStamp& time_stamp_;
    enumeration::ArmorIdFlag ids_;
    double bullet_speed_;
};

TargetSnapshotManager::TargetSnapshotManager(const std::string& config_path,
    const std::unordered_map<enumeration::ArmorIdFlag, std::unique_ptr<LiveTarget>>&
        live_target_map,
    const data::TimeStamp& time_stamp)
    : pimpl_(std::make_unique<Impl>(config_path, live_target_map, time_stamp)) { }
TargetSnapshotManager::~TargetSnapshotManager() = default;

const enumeration ::ArmorIdFlag& TargetSnapshotManager::GetId() const { return pimpl_->GetId(); }
std ::shared_ptr<interfaces::IArmorInGimbalControl> TargetSnapshotManager::Predictor(
    const data::TimeStamp& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}
}
