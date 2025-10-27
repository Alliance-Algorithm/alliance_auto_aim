#include "target_snapshot_manager.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "../in_gimbal_control_armor.hpp"
#include "../live_target_manager/live_target.hpp"
#include "aim_solver.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/enum_tools.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshotManager::Impl {
public:
    Impl(const std::string& config_path, const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
            live_target_map,
        const std::time_t& now)
        : aim_solver_(std::make_unique<predictor::AimingSolver>(config_path))
        , snapshots_(BuildSnapshots(live_target_map))
        , now_(now)
        , ids_(id)
        , gimbal_command_({ std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN() }) {

        auto yaml = YAML::LoadFile(config_path);

        decision_speed_        = yaml["decision_speed"].as<double>();
        high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
        low_speed_delay_time_  = yaml["low_speed_delay_time"].as<double>();
        bullet_speed_          = yaml["bullet_spped"].as<double>();
    }

    const enumeration::ArmorIdFlag& GetId() const { return ids_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std::time_t& time_stamp) const {

        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;
        // TODO:time_delay
        for (const auto& [id, snapshot] : snapshots_) {
            auto ekf_x = snapshot.GetEkfX();
            const auto delay_time =
                ekf_x[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
            const auto dt = control_delay_time_ + delay_time; // TODO:add delta(now)?

            auto aim_solution = aim_solver_->SolveAimSolution(
                std::make_unique<TargetSnapshot>(snapshot), bullet_speed_, dt);

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
    double bullet_speed_;
    mutable GimbalCommand gimbal_command_;
    double decision_speed_;
    double high_speed_delay_time_;
    double low_speed_delay_time_;
    double control_delay_time_;
};

TargetSnapshotManager::TargetSnapshotManager(const std::string& config_path,
    const enumeration::ArmorIdFlag& id,
    const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
        live_target_map,
    const std::time_t& now)
    : pimpl_(std::make_unique<Impl>(config_path, id, live_target_map, now)) { }
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
