#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <memory>
#include <optional>

#include <print>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "../predictor/car_predictor/car_predictor.hpp"
#include "aim_point_chooser.hpp"
#include "data/time_stamped.hpp"
#include "tongji/predictor/kalman_filter/extended_kalman_filter.hpp"
#include "tongji/predictor/kalman_filter/predict_model.hpp"
#include "trajectory.hpp"

namespace world_exe::tongji::fire_control {

struct AimSolution {
    bool valid;
    double yaw;
    double pitch;
    Eigen::Vector3d aim_point;
    double horizon_distance = 0; // 无人机专有
};

class AimingSolver {
public:
    using PredictorModel = predictor::EKFModel<11, 4>;
    using EKF            = predictor::ExtendedKalmanFilter<PredictorModel>;

    explicit AimingSolver(const std::string& config_path, const double& gravity = 9.7833)
        : aim_point_chooser_(std::make_unique<AimPointChooser>(config_path))
        , g_(gravity) {

        auto yaml     = YAML::LoadFile(config_path);
        yaw_offset_   = yaml["yaw_offset"].as<double>() / 57.3;   // degree to rad
        pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3; // degree to rad
        bullet_speed_ = yaml["bullet_speed"].as<double>();
    }

    AimSolution SolveAimSolution(std::shared_ptr<interfaces::IPredictor> const& snapshot,
        data::TimeStamp const& time_stamp, std::chrono::milliseconds control_delay) {

        // 迭代求解飞行时间
        // (最多10次，收敛条件：相邻两次fly_time差 <0.001)
        double prev_fly_time_s = 0;
        Eigen::Vector3d final_aim_point;
        TrajectoryResult final_trajectory;
        bool converged = false;
        // HACK:不同击打点影响飞行时间的迭代，需要根据整车的状态（转速和坐标）来选择击打点，不得已将指针转换为派生类
        auto snapshot_derived = std::dynamic_pointer_cast<predictor::CarPredictor>(snapshot);
        if (!snapshot_derived)
            throw std::runtime_error("Failed to cast snapshot to CarPredictor. Unexpected object "
                                     "type.");

        // 预测目标在未来 dt时间后的位置
        for (int i = 0; i < 10; ++i) {
            const auto& dt = prev_fly_time_s + (double)(control_delay).count() / 1000.;
            const auto& armors =
                snapshot->Predictor(time_stamp + data::TimeStamp::from_seconds(dt));

            // const auto& armors_to_view = armors->GetArmors(snapshot->GetId());
            // armors_to_view_            = std::make_shared<predictor::InGimbalControlArmor>(
            //     armors_to_view, time_stamp + data::TimeStamp::from_seconds(dt));

            const auto& aim_point = SelectPredictedAim(
                snapshot_derived->GetPredictedX(time_stamp), armors->GetArmors(snapshot->GetId()), snapshot->GetId());

            if (!aim_point.has_value()) {
                std::println("no valid aim point");
                continue;

            } // failed: no valid aim point

            const auto traj = SolveTrajectory(aim_point.value(), bullet_speed_);
            if (!traj.has_value()) {
                std::println("trajectory unsolvable");
                continue;
            }

            if (i > 0 && std::abs(traj->fly_time - prev_fly_time_s) < 0.001) {
                final_aim_point  = *aim_point;
                final_trajectory = *traj;
                converged        = true;
                break;
            }
            prev_fly_time_s = traj->fly_time;
        }

        if (!converged) {
            std::println("trajectory diverse");

            return { false, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), { },
                0 }; // failed: trajectory did not converge
        }

        const auto xyz     = final_aim_point;
        const double yaw   = std::atan2(xyz.y(), xyz.x()) + yaw_offset_;
        const double pitch = (final_trajectory.pitch + pitch_offset_);

        return { true, yaw, pitch, final_aim_point };
    }

    // auto GetArmorsToView() -> std::shared_ptr<interfaces::IArmorInGimbalControl> {
    //     return armors_to_view_;
    // }

private:
    std::optional<Eigen::Vector3d> SelectPredictedAim(const EKF::XVec& ekf_x,
        const std::vector<data::ArmorGimbalControlSpacing>& armors, const CarIDFlag& id) const {
        const auto& [selectable, aim_point_in_gimbal] =
            aim_point_chooser_->ChooseAimArmor(ekf_x, armors, id);

        if (!selectable) return std::nullopt;
        return aim_point_in_gimbal.position;
    }

    std::optional<TrajectoryResult> SolveTrajectory(
        const Eigen::Vector3d& xyz, const double& bullet_speed) const {
        double d    = std::hypot(xyz.x(), xyz.y());
        auto result = TrajectorySolver::SolveTrajectory(bullet_speed, d, xyz.z(), g_);

        if (!result.solvable) {
            std::println("solve trajectory failed: d={}, z={},speed={}", d, xyz.z(), bullet_speed);
        }

        return result.solvable ? std::optional { result } : std::nullopt;
    }

    double yaw_offset_, pitch_offset_;
    double bullet_speed_;
    const double g_;
    // std::shared_ptr<interfaces::IArmorInGimbalControl> armors_to_view_;

    std::unique_ptr<AimPointChooser> aim_point_chooser_;
};
}
