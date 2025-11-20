#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "../predictor/car_predictor/car_predictor.hpp"
#include "aim_point_chooser.hpp"
#include "data/time_stamped.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
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
        Eigen::Affine3d const& transform_gimbal2muzzle, data::TimeStamp const& time_stamp,
        std::chrono::milliseconds const& control_delay) {

        // 迭代求解飞行时间
        // (最多10次，收敛条件：相邻两次fly_time差 <0.001)
        double prev_fly_time_s = 0;
        Eigen::Vector3d final_aim_point;
        TrajectoryResult final_trajectory;
        bool converged   = false;
        auto fire_origin = transform_gimbal2muzzle.inverse().translation();

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

            const auto& armors_in_gimbal = armors->GetArmors(snapshot->GetId());

            armors_view_ = std::make_shared<predictor::InGimbalControlArmor>(
                armors_in_gimbal, time_stamp + data::TimeStamp::from_seconds(dt));

            const auto& aim_point = SelectPredictedAim(
                snapshot_derived->GetPredictedX(time_stamp), armors_in_gimbal, snapshot->GetId());

            if (!aim_point.has_value()) {
                continue;
            } // failed: no valid aim point

            auto aim_vector = *aim_point - fire_origin;
            // std::cout << "aim_vector:(" << aim_vector.x() << "," << aim_vector.y() << ","
            //           << aim_vector.z() << ")"  << "   fire_origin:(" << fire_origin.x() << ","
            //           << fire_origin.y() << ","
            //       << fire_origin.z() << ")" << std::endl;

            const auto traj = SolveTrajectory(aim_vector, bullet_speed_);
            if (!traj.has_value()) {
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
            std::cout << "trajectory did not converge" << std::endl;
            return { false, { }, 0 }; // failed: trajectory did not converge
        }

        const auto vec     = final_aim_point - fire_origin;
        const double yaw   = (std::atan2(vec.y(), vec.x()) + yaw_offset_);
        const double pitch = (final_trajectory.pitch + pitch_offset_);
        return { true, yaw, pitch, final_aim_point };
    }

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> GetArmorsSnapshot() {
        return armors_view_;
    }

private:
    std::shared_ptr<predictor::InGimbalControlArmor> armors_view_;

    template <std::ranges::range T>
    std::optional<Eigen::Vector3d> SelectPredictedAim(
        const EKF::XVec& ekf_x, const T& armors, const CarIDFlag& id) const {
        const auto& [selectable, aim_point_in_gimbal] =
            aim_point_chooser_->ChooseAimArmor(ekf_x, armors, id);

        if (!selectable) return std::nullopt;
        return aim_point_in_gimbal.position;
    }

    std::optional<TrajectoryResult> SolveTrajectory(
        const Eigen::Vector3d& vec, const double& bullet_speed) const {
        double d    = std::hypot(vec.x(), vec.y());
        auto result = TrajectorySolver::SolveTrajectory(bullet_speed, d, vec.z(), g_);

        if (!result.solvable) {
            std::cout << "solve trajectory failed: d=" << d << " z=" << vec.z()
                      << "speed=" << bullet_speed << std::endl;
        }

        return result.solvable ? std::optional { result } : std::nullopt;
    }

    double yaw_offset_, pitch_offset_;
    double bullet_speed_;
    const double g_;

    std::unique_ptr<AimPointChooser> aim_point_chooser_;
};
}
