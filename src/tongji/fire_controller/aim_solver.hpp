#pragma once

#include <cmath>
#include <ctime>
#include <memory>
#include <optional>
#include <utility>

#include "tongji/fire_controller/aim_point_chooser.hpp"
#include "tongji/fire_controller/trajectory.hpp"
#include "tongji/predictor/target_snapshot.hpp"

namespace world_exe::tongji::fire_control {

struct AimSolution {
    bool valid;
    double yaw;
    double pitch;
    Eigen::Vector4d aim_point;   // 最终瞄准点（世界坐标 + 装甲板yaw）
    double horizon_distance = 0; // 无人机专有
};

class AimingSolver {
public:
    AimingSolver( const double& bullet_speed,
        const double& yaw_offset, const double& pitch_offset, const double& gravity = 9.7833)
        : aim_point_chooser_(std::make_unique<AimPointChooser>())
        , bullet_speed_(bullet_speed)
        , yaw_offset_(yaw_offset / 57.3)     // degree to rad
        , pitch_offset_(pitch_offset / 57.3) // degree to rad
        , g_(gravity) { }

    void UpdateSnapshot(std::unique_ptr<predictor::TargetSnapshot> snapshot) {
        snapshot_ = std::move(snapshot);
    }

    AimSolution SolveAimSolution(const double& time_delay) {
        if (!snapshot_) return { false, 0, 0, {}, 0 };

        // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
        double prev_fly_time = 0;
        Eigen::Vector4d final_aim_point;
        TrajectoryResult final_trajectory;
        bool converged = false;

        // 预测目标在未来 dt时间后的位置
        for (int i = 0; i < 10; ++i) {
            double dt      = time_delay + prev_fly_time;
            const auto aim = SelectPredictedAim(dt);
            if (!aim.has_value()) return { false, 0, 0, {}, 0 }; // failed: no valid aim point

            const auto traj = SolveTrajectory(aim->head(3));
            if (!traj.has_value()) return { false, 0, 0, {}, 0 }; // failed: trajectory unsolvable

            if (i > 0 && std::abs(traj->fly_time - prev_fly_time) < 0.001) {
                final_aim_point  = *aim;
                final_trajectory = *traj;
                converged        = true;
                break;
            }
            prev_fly_time = traj->fly_time;
        }
        if (!converged) return { false, 0, 0, {}, 0 }; // failed: trajectory did not converge

        const auto xyz     = final_aim_point.head(3);
        const double yaw   = std::atan2(xyz.y(), xyz.x()) + yaw_offset_;
        const double pitch = -(final_trajectory.pitch + pitch_offset_);
        return { true, yaw, pitch, final_aim_point };
    }

private:
    std::optional<Eigen::Vector4d> SelectPredictedAim(double dt) const {
        const auto& [selectable, aim_point] = aim_point_chooser_->ChooseAimArmor(
            snapshot_->Predict(dt), snapshot_->GetPredictedXYZAList(dt), snapshot_->GetID());
        return selectable ? std::optional { aim_point } : std::nullopt;
    }

    std::optional<TrajectoryResult> SolveTrajectory(const Eigen::Vector3d& xyz) const {
        double d    = std::hypot(xyz.x(), xyz.y());
        auto result = TrajectorySolver::SolveTrajectory(bullet_speed_, d, xyz.z(), g_);
        return result.solvable ? std::optional { result } : std::nullopt;
    }

    double bullet_speed_; // m/s
    double yaw_offset_, pitch_offset_;
    const double g_;

    std::unique_ptr<AimPointChooser> aim_point_chooser_;
    std::unique_ptr<predictor::TargetSnapshot> snapshot_;
};
}