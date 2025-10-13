#include "target_snapshot_manager.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

#include "../in_gimbal_control_armor.hpp"
#include "../live_target_manager/live_target.hpp"
#include "aim_solver.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/enum_tools.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshotManager::Impl {
public:
    Impl(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
            live_target_map,
        const std::time_t& now, const double& bullet_speed, const double& yaw_offset,
        const double& pitch_offset)
        : aim_solver_(std::make_unique<predictor::AimingSolver>(yaw_offset, pitch_offset))
        , snapshots_(BuildSnapshots(live_target_map))
        , now_(now)
        , ids_(id)
        , bullet_speed_(bullet_speed)
        , gimbal_command_({ std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN() }) { }

    const enumeration::ArmorIdFlag& GetId() const { return ids_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std::time_t& time_stamp) const {

        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (const auto& [id, snapshot] : snapshots_) {
            auto aim_solution =
                aim_solver_->SolveAimSolution(std::make_unique<TargetSnapshot>(snapshot),
                    bullet_speed_, snapshot.GetTimeStamp().GetTimeStamp());

            if (!aim_solution.valid) continue;

            auto target_pos = aim_solution.aim_point.head<3>();
            auto armor_yaw  = aim_solution.aim_point[3];
            result.emplace(id,
                std::vector<data::ArmorGimbalControlSpacing> {
                    data::ArmorGimbalControlSpacing { id, target_pos,
                        util::math::euler_to_quaternion(armor_yaw, 15. / 180. * CV_PI, 0) } });

            gimbal_command_.yaw   = aim_solution.yaw;
            gimbal_command_.pitch = aim_solution.pitch;
        }
        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

    auto GetGimbalCommand() const -> GimbalCommand const { return gimbal_command_; }

private:
    static std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> BuildSnapshots(
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>& input) {
        std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> result;
        for (const auto& [id, target] : input) {
            if (target) {
                result.emplace(id, TargetSnapshot(*target));
            }
        }
        return result;
    }

    std::unique_ptr<predictor::AimingSolver> aim_solver_;
    const std::unordered_map<enumeration::ArmorIdFlag, TargetSnapshot> snapshots_;
    const std::time_t& now_;
    const enumeration::ArmorIdFlag ids_;
    const double bullet_speed_;
    mutable GimbalCommand gimbal_command_;
};

TargetSnapshotManager::TargetSnapshotManager(const enumeration::ArmorIdFlag& id,
    const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
        live_target_map,
    const std::time_t& now, const double& bullet_speed, const double& yaw_offset,
    const double& pitch_offset)
    : pimpl_(std::make_unique<Impl>(
          id, live_target_map, now, bullet_speed, yaw_offset, pitch_offset)) { }
TargetSnapshotManager::~TargetSnapshotManager() = default;

const enumeration ::ArmorIdFlag& TargetSnapshotManager::GetId() const { return pimpl_->GetId(); }
std ::shared_ptr<interfaces::IArmorInGimbalControl> TargetSnapshotManager::Predictor(
    const std ::time_t& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}

auto TargetSnapshotManager::GetGimbalCommand() const -> GimbalCommand const {
    return pimpl_->GetGimbalCommand();
}
}