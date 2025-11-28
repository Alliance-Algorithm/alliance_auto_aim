#pragma once

#include "data/time_stamped.hpp"
#include <memory>

#include <Eigen/Dense>

namespace world_exe::tongji::planner {

constexpr double DT        = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON      = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>; // delta_yaw, yaw_vel, delta_pitch, pitch_vel

struct Plan {
    bool control;
    bool fire;
    double target_yaw;
    double target_pitch;
    double yaw;
    double yaw_vel;
    double yaw_acc;
    double pitch;
    double pitch_vel;
    double pitch_acc;
};

struct PlanInfo {
    Trajectory trajectory;
    data::TimeStamp time_stamp;
    double current_yaw;
};

class Planner final {
public:
    explicit Planner(const std::string& config_path);
    ~Planner();

    auto Plan(Trajectory const& traj, double yaw0) -> struct Plan;

    Planner(const Planner&)                = delete;
    Planner& operator=(const Planner&)     = delete;
    Planner(Planner&&) noexcept            = default;
    Planner& operator=(Planner&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}