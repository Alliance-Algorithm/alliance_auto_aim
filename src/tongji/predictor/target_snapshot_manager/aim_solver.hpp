#pragma once

#include <cmath>
#include <ctime>
#include <memory>
#include <optional>

#include <yaml-cpp/yaml.h>

#include "aim_point_chooser.hpp"
#include "target_snapshot.hpp"
#include "trajectory.hpp"

namespace world_exe::tongji::predictor {

using TargetSnapshot = predictor::TargetSnapshot;

struct AimSolution {
    bool valid;
    double yaw;
    double pitch;
    Eigen::Vector4d aim_point;   // 最终瞄准点（世界坐标 + 装甲板yaw）
    double horizon_distance = 0; // 无人机专有
};

class AimingSolver {
public:
    AimingSolver(const std::string& config_path, const double& gravity = 9.7833)
        : aim_point_chooser_(std::make_unique<predictor::AimPointChooser>(config_path))
        , g_(gravity) {

        auto yaml     = YAML::LoadFile(config_path);
        yaw_offset_   = yaml["yaw_offset"].as<double>() / 57.3;   // degree to rad
        pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3; // degree to rad
    }

    AimSolution SolveAimSolution(const std::shared_ptr<TargetSnapshot>& snapshot,
        const double& bullet_speed, const double& time_delay) {
        if (!snapshot) return { false, 0, 0, { }, 0 };

        // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
        double prev_fly_time = 0;
        Eigen::Vector4d final_aim_point;
        TrajectoryResult final_trajectory;
        bool converged = false;

        // 预测目标在未来 dt时间后的位置
        for (int i = 0; i < 10; ++i) {
            double dt            = time_delay + prev_fly_time;
            const auto aim_point = SelectPredictedAim(snapshot, dt);
            if (!aim_point.has_value())
                return { false, 0, 0, { }, 0 }; // failed: no valid aim point

            const auto traj = SolveTrajectory(aim_point->head<3>(), bullet_speed);
            if (!traj.has_value()) return { false, 0, 0, { }, 0 }; // failed: trajectory unsolvable

            if (i > 0 && std::abs(traj->fly_time - prev_fly_time) < 0.001) {
                final_aim_point  = *aim_point;
                final_trajectory = *traj;
                converged        = true;
                break;
            }
            prev_fly_time = traj->fly_time;
        }
        if (!converged) return { false, 0, 0, { }, 0 }; // failed: trajectory did not converge

        const auto xyz     = final_aim_point.head<3>();
        const double yaw   = std::atan2(xyz.y(), xyz.x()) + yaw_offset_;
        const double pitch = -(final_trajectory.pitch + pitch_offset_);
        return { true, yaw, pitch, final_aim_point };
    }

private:
    std::optional<Eigen::Vector4d> SelectPredictedAim(
        const std::shared_ptr<TargetSnapshot>& snapshot, const double& dt) const {
        const auto& [selectable, aim_point_in_gimbal] = aim_point_chooser_->ChooseAimArmor(
            snapshot->Predict(dt), snapshot->GetPredictedXYZAList(dt), snapshot->GetID());

        if (!selectable) return std::nullopt;
        return aim_point_in_gimbal;
    }

    std::optional<TrajectoryResult> SolveTrajectory(
        const Eigen::Vector3d& xyz, const double& bullet_speed) const {
        double d    = std::hypot(xyz.x(), xyz.y());
        auto result = TrajectorySolver::SolveTrajectory(bullet_speed, d, xyz.z(), g_);
        return result.solvable ? std::optional { result } : std::nullopt;
    }

    double yaw_offset_, pitch_offset_;
    const double g_;

    std::unique_ptr<predictor::AimPointChooser> aim_point_chooser_;
};
}