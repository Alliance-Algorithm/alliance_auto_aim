#pragma once

#include <cmath>
#include <ctime>
#include <memory>
#include <utility>

#include "tongji/fire_controller/aim_point_chooser.hpp"
#include "tongji/fire_controller/trajectory.hpp"
#include "tongji/predictor/target_snapshot.hpp"

namespace world_exe::tongji::fire_control {

struct ControlCommand {
    bool solvable;
    double yaw;
    double pitch;
    double horizon_distance = 0; // 无人机专有
};

class FireControlSolver {
public:
    FireControlSolver(std::unique_ptr<predictor::TargetSnapshot> snapshot,
        const double& bullet_speed, const double& yaw_offset, const double& pitch_offset,
        const double& gravity = 9.7833)
        : snapshot_(std::move(snapshot))
        , aim_point_chooser_(std::make_unique<AimPointChooser>())
        , bullet_speed_(bullet_speed)
        , g_(gravity) { }

    ControlCommand GenerateCommand() {
        const auto& [valid0, aim_point0] = aim_point_chooser_->ChooseAimArmor(
            snapshot_->GetEkfX(), snapshot_->GetXYZAList(0), snapshot_->GetID());
        if (!valid0) return { false, 0, 0 };

        const auto& xyz0       = aim_point0.head(3);
        const auto& d0         = std::sqrt(xyz0.x() * xyz0.x() + xyz0.y() * xyz0.y());
        const auto trajectory0 = TrajectorySolver::SolveTrajectory(bullet_speed_, d0, xyz0[2], g_);

        // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
        bool converged       = false;
        double prev_fly_time = trajectory0.fly_time;

        // 预测目标在 future + prev_fly_time 时刻的位置
        Eigen::Vector4d final_aim_point;
        auto current_trajectory = trajectory0;

        for (int i = 0; i < 10; ++i) {
            auto dt                        = prev_fly_time;
            const auto& [valid, aim_point] = aim_point_chooser_->ChooseAimArmor(
                snapshot_->Predict(dt), snapshot_->GetXYZAList(dt), snapshot_->GetID());

            if (!valid) return { false, 0., 0. };

            const auto& xyz    = aim_point.head(3);
            const auto& d      = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
            current_trajectory = TrajectorySolver::SolveTrajectory(bullet_speed_, d, xyz.z(), g_);

            if (!current_trajectory.solvable) return { false, 0., 0. };

            // 检查收敛条件
            if (std::abs(current_trajectory.fly_time - prev_fly_time) < 0.001) {
                converged       = true;
                final_aim_point = aim_point;
                break;
            }
            prev_fly_time = current_trajectory.fly_time;
        }

        if (!converged) return { false, 0., 0. };

        // 计算最终角度
        Eigen::Vector3d final_xyz = final_aim_point.head(3);
        const double yaw          = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
        const double pitch =
            -(current_trajectory.pitch + pitch_offset_); // 世界坐标系下pitch向上为负
        return { true, yaw, pitch };
    }

private:
    double bullet_speed_;
    double yaw_offset_, pitch_offset_;
    const double g_;
    std::unique_ptr<predictor::TargetSnapshot> snapshot_;
    std::unique_ptr<AimPointChooser> aim_point_chooser_;
};
}