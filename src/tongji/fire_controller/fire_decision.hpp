#pragma once

#include <cmath>
#include <cstdlib>

#include "tongji/predictor/target_snapshot_manager/target_snapshot_manager.hpp"

namespace world_exe::tongji::fire_control {

class FireDecision {
public:
    explicit FireDecision(const bool& auto_fire, const double& first_tolerance = 5,
        const double& second_tolerance = 2, const double& judge_distance = 3)
        : auto_fire_(auto_fire)
        , last_gimbal_command_({ std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN() })
        , first_tolerance_(first_tolerance)
        , second_tolerance_(second_tolerance)
        , judge_distance_(judge_distance) { }

    bool ShouldFire(
        predictor::GimbalCommand gimbal_command, const Eigen::Vector3d& valid_target_pos) {

        if (!auto_fire_) return false;
        const auto& tolerance = std::sqrt(valid_target_pos.x() * valid_target_pos.x()
                                    + valid_target_pos.y() * valid_target_pos.y())
                > judge_distance_
            ? second_tolerance_
            : first_tolerance_;

        if (std::abs(last_gimbal_command_.yaw - gimbal_yaw_)
                < tolerance * 2 // 此时认为command突变不应该射击
            && std::abs(gimbal_yaw_ - last_gimbal_command_.yaw) < tolerance) {
            last_gimbal_command_ = gimbal_command;
            return true;
        }
        last_gimbal_command_ = gimbal_command;
        return false;
    }

private:
    bool auto_fire_;
    predictor::GimbalCommand last_gimbal_command_;

    double gimbal_yaw_;

    double first_tolerance_ { 5 };  // 近距离射击容差，degree
    double second_tolerance_ { 2 }; // 远距离射击容差，degree
    double judge_distance_ { 3 };   // 距离判断阈值
};

}