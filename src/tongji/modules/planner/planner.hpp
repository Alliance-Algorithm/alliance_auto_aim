#pragma once

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
    float target_yaw;
    float target_pitch;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
};

class Planner final {
public:
    explicit Planner(const std::string& config_path);
    ~Planner();

    auto Plan(const double yaw0) -> const Plan;
    auto SetTrajectory(const Trajectory trajectory) -> void;

    Planner(const Planner&)                = delete;
    Planner& operator=(const Planner&)     = delete;
    Planner(Planner&&) noexcept            = default;
    Planner& operator=(Planner&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
