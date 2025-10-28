#include "target_snapshot_manager.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "../in_gimbal_control_armor.hpp"
#include "../live_target_manager/live_target.hpp"
#include "aim_solver.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/enum_tools.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshotManager::Impl {
public:
    Impl(const std::string& config_path, const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>>&
            live_target_map,
        const data::TimeStamp& now)
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
        bullet_speed_          = yaml["bullet_speed"].as<double>();
    }

    const enumeration::ArmorIdFlag& GetId() const { return ids_; }

    /*
    通过aim_solver_->SolveAimSolution()来解算瞄准击打点
    aim_solution.aim_point，从而得到返回值std::shared_ptr<interfaces::IArmorInGimbalControl>
    所以此处进行飞行时间的迭代
    但飞行时间的迭代需要考虑到时间延迟（图像传输这些等），这个参数在这个接口中体现：
    data::FireControl CalculateTarget(const std::time_t& time_duration)

    很显然为了实现这个接口，得到返回值std::shared_ptr<interfaces::IArmorInGimbalControl>，
    我需要进行飞行时间的迭代，但在这个接口中我没法把实际的时间延迟传入（我认为这个值是动态的，从上层调用它的地方传入）
    v1的版本，传入的是 时间间隔（延迟），但这个接口的含义是传入
    某个时间点返回装甲板信息，而非时间间隔

    如果是在data::FireControl CalculateTarget(const std::time_t&
    time_duration)这里进行飞行时间的迭代，那么已经得到了data::FireControl，也就是最后的命令，
    那就不需要这个接口来得到中间量std::shared_ptr<interfaces::IArmorInGimbalControl>

    */
    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const {

        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;
        // TODO:time_delay
        for (const auto& [id, snapshot] : snapshots_) {
            auto ekf_x = snapshot.GetEkfX();
            const auto delay_time =
                ekf_x[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
            const auto dt = control_delay_time_ + delay_time;

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

    /*
    飞行时间的迭代得到一些有用的信息，比如说，云台控制指令GimbalCommand，
    需要保存一下，后续需要和当前云台yaw来分析这个指令有没有突变，但是有const约束，虽然好像问题不大
    */
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
    const data::TimeStamp& now_;
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
    const data::TimeStamp& now)
    : pimpl_(std::make_unique<Impl>(config_path, id, live_target_map, now)) { }
TargetSnapshotManager::~TargetSnapshotManager() = default;

const enumeration ::ArmorIdFlag& TargetSnapshotManager::GetId() const { return pimpl_->GetId(); }
std ::shared_ptr<interfaces::IArmorInGimbalControl> TargetSnapshotManager::Predictor(
    const data::TimeStamp& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}

auto TargetSnapshotManager::GetGimbalCommand() const -> GimbalCommand const {
    return pimpl_->GetGimbalCommand();
}
}
