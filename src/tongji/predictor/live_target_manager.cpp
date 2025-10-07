#include "live_target_manager.hpp"

#include <cstdint>
#include <memory>
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

        for (auto id : util::enumeration::ExpandArmorIdFlags(flag)) {
            auto it = targets_.find(id);
            if (it != targets_.end() && it->second && it->second->IsConverged()) {
                auto spacings = it->second->GetArmorGimbalControlSpacings();
                result[id]    = std::move(spacings);
            }
        }

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

    auto GetLiveTargetIDs() -> enumeration::CarIDFlag const {
        enumeration::CarIDFlag result = enumeration::CarIDFlag::None;
        for (const auto& [id, target] : targets_) {
            if (target && target->IsConverged()) {
                result = static_cast<enumeration::CarIDFlag>(
                    static_cast<uint32_t>(result) | static_cast<uint32_t>(id));
            }
        }
        return result;
    }

private:
    void UpdateTargets(
        const std::shared_ptr<interfaces::IPreDictorUpdatePackage>& data, double dt) {
        const Eigen::Affine3d transform       = data->GetTransform();
        const Eigen::Matrix3d rotation_matrix = transform.linear();
        const Eigen::Quaterniond rotation_quat(rotation_matrix);

        const auto& armors_interface = data->GetArmors();
        for (int i; i < 8; i++) {
            auto id = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);
            const auto& armors = armors_interface->GetArmors(id);

            if (armors.empty()) continue;

            if (!HasTarget(id)) {
                const auto& armor                       = armors.front();
                const Eigen::Vector3d position_in_world = transform * armor.position;
                const Eigen::Vector3d ypr_in_world = rotation_matrix.eulerAngles(2, 1, 0); // ZYX

                auto target = std::make_shared<LiveTarget>(position_in_world, ypr_in_world, id);
                RegisterTarget(id, target);
            }

            auto target = GetTarget(id);
            for (const auto& armor : armors) {
                const Eigen::Vector3d position_in_world = transform * armor.position;
                const Eigen::Vector3d ypr =
                    util::math::quaternion_to_euler(Eigen::Quaterniond(rotation_matrix), 2, 1, 0);

                target->Update(dt, position_in_world, ypr, util::math::xyz2ypd(position_in_world));
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