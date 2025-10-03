#pragma once

#include <cmath>
#include <cstdlib>

#include "tongji/fire_controller/gimbal_angle_solver.hpp"

namespace world_exe::tongji::fire_control {

class FireJudge {
public:
    explicit FireJudge(const bool& auto_fire, const double& first_tolerance = 5,
        const double& second_tolerance = 2, const double& judge_distance = 3)
        : auto_fire_(auto_fire)
        , last_command_ { false, 0, 0 }
        , first_tolerance_(first_tolerance)
        , second_tolerance_(second_tolerance)
        , judge_distance_(judge_distance) { }

    bool ShouldFire(const GimbalCommand& command, const Eigen::Vector3d& valid_target_pos,
        const Eigen::Vector3d& gimbal_pos) {

        if (!command.valid || !auto_fire_) return false;
        const auto& tolerance = std::sqrt(valid_target_pos.x() * valid_target_pos.x()
                                    + valid_target_pos.y() * valid_target_pos.y())
                > judge_distance_
            ? second_tolerance_
            : first_tolerance_;

        if (std::abs(last_command_.yaw - command.yaw)
                < tolerance * 2 // 此时认为command突变不应该射击
            && std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance && command.valid) {
            last_command_ = command;
            return true;
        }
        last_command_ = command;
        return false;
    }

private:
    bool auto_fire_;
    GimbalCommand last_command_;

    double first_tolerance_ { 5 };  // 近距离射击容差，degree
    double second_tolerance_ { 2 }; // 远距离射击容差，degree
    double judge_distance_ { 3 };   // 距离判断阈值
};

}