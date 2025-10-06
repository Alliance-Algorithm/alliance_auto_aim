#include "live_target_manager.hpp"

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/predictor/live_target.hpp"
#include "tongji/predictor/target_snapshot_manager.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager::Impl {
public:
    Impl(double timeout_sec = 0.1)
        : timeout_sec_(timeout_sec) { }

    void RegisterTarget(enumeration::ArmorIdFlag id, std::shared_ptr<LiveTarget> target) {
        targets_[id] = std::move(target);
    }

    void RemoveTarget(enumeration::ArmorIdFlag id) { targets_.erase(id); }
    bool HasTarget(enumeration::ArmorIdFlag id) const { return targets_.count(id) > 0; }
    std::shared_ptr<LiveTarget> GetTarget(enumeration::ArmorIdFlag id) const {
        auto it = targets_.find(id);
        return (it != targets_.end()) ? it->second : nullptr;
    }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& flag, const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        throw std::runtime_error("Not implemented");
        // TODO
        // for (auto id : util::enumeration::ExpandArmorIdFlags(flag)) {
        //     auto it = targets_.find(id);
        //     if (it != targets_.end() && it->second && it->second->IsConverged()) {
        //         result[id].emplace_back(it->second->GetArmorGimbalControlSpacings());
        //     }
        // }

        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

    std::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration::ArmorIdFlag& flag) const {
        std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> snapshot_map;

        for (auto id : util::enumeration::ExpandArmorIdFlags(flag)) {
            auto it = targets_.find(id);
            if (it != targets_.end() && it->second && it->second->IsConverged()) {
                snapshot_map[id] = it->second;
            }
        }

        return std::make_shared<TargetSnapshotManager>(flag, snapshot_map);
    }

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data, double dt) {
        const auto now = predictor::TimeStamp(std::time(nullptr));
        RemoveLostTargets(now);
        UpdateTargets(data, dt);
    }

private:
    void UpdateTargets(
        const std::shared_ptr<interfaces::IPreDictorUpdatePackage>& data, double dt) {
        const auto& transform          = data->GetTransform();
        const auto& rotation_transform = Eigen::Quaterniond(transform.linear());

        for (const auto& [id, target] : targets_) {
            const auto armors = data->GetArmors()->GetArmors(id);

            if (armors.empty()) continue;

            for (auto armor : armors) {
                const auto& armor_in_world_position  = transform * armor.position;
                const auto& armor_in_world_oritaiton = rotation_transform * armor.orientation;
                target->Update(dt, armor_in_world_position,
                    util::math::quaternion_to_euler(armor_in_world_oritaiton, 2, 1, 0),
                    util::math::xyz2ypd(armor_in_world_position));
            }
        }
    }

    void RemoveLostTargets(const predictor::TimeStamp& now) {
        for (auto it = targets_.begin(); it != targets_.end();) {
            if (IsTargetLost(it->second, now)) {
                it = targets_.erase(it);
            } else {
                ++it;
            }
        }
    }

    bool IsTargetLost(
        const std::shared_ptr<LiveTarget>& target, const predictor::TimeStamp& now) const {
        return now.SecondsSince(target->LastSeen()) > timeout_sec_;
    }

    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> targets_;
    const double timeout_sec_ { 0.1 };
};

LiveTargetManager::LiveTargetManager()  = default;
LiveTargetManager::~LiveTargetManager() = default;

std ::shared_ptr<interfaces ::IArmorInGimbalControl> LiveTargetManager::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}

std ::shared_ptr<interfaces::IPredictor> LiveTargetManager::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

}