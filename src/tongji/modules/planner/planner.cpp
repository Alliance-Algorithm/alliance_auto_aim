#include "planner.hpp"

#include <cmath>

#include <vector>
#include <yaml-cpp/yaml.h>

#include "tinympc/tiny_api.hpp"
#include "tongji/utils/math.hpp"
namespace world_exe::tongji::planner {

class Planner::Impl {
public:
    Impl(const std::string& config_path) {
        auto yaml    = YAML::LoadFile(config_path);
        fire_thresh_ = yaml["fire_thresh"].as<double>();
        SetYawSolver(config_path);
        SetPitchSolver(config_path);
    }
    ~Impl() = default;

    auto Plan(double yaw0) -> const struct Plan {
        // Solve yaw
        Eigen::Vector2d x0;
        x0 << trajectory_(0, 0), trajectory_(1, 0);
        tiny_set_x0(yaw_solver_, x0);

        yaw_solver_->work->Xref =
            trajectory_.block(0, 0, 2, HORIZON); 
        tiny_solve(yaw_solver_);

        //  Solve pitch
        x0 << trajectory_(2, 0), trajectory_(3, 0);
        tiny_set_x0(pitch_solver_, x0);

        pitch_solver_->work->Xref = trajectory_.block(2, 0, 2, HORIZON);
        tiny_solve(pitch_solver_);

        struct Plan plan;
        plan.control = true;
        plan.yaw     = utils::math::clamp_pm_pi(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
        plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
        plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

        plan.pitch     = pitch_solver_->work->x(0, HALF_HORIZON);
        plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
        plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

        plan.fire = std::hypot(trajectory_(0, HALF_HORIZON + shoot_offset_)
                            - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
                        trajectory_(2, HALF_HORIZON + shoot_offset_)
                            - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_))
            < fire_thresh_;
        return plan;
    }

    auto
    SetTrajectory(const Trajectory trajectory) -> void {
        trajectory_ = std::move(trajectory);
    }

private:
    auto SetYawSolver(const std::string& config_path) -> void {
        auto yaml        = YAML::LoadFile(config_path);
        auto max_yaw_acc = yaml["max_yaw_acc"].as<double>();
        auto Q_yaw       = yaml["Q_yaw"].as<std::vector<double>>();
        auto R_yaw       = yaml["R_yaw"].as<std::vector<double>>();

        Eigen::Matrix<double, 2, 2> A { { 1, DT }, { 0, 1 } };
        Eigen::Matrix<double, 2, 1> B { { 0 }, { DT } };
        Eigen::Vector2d f { { 0, 0 } };
        Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
        Eigen::Matrix<double, 1, 1> R(R_yaw.data());
        tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        yaw_solver_->settings->max_iter = 10;
    }

    auto SetPitchSolver(const std::string& config_path) -> void {
        auto yaml          = YAML::LoadFile(config_path);
        auto max_pitch_acc = yaml["max_pitch_acc"].as<double>();
        auto Q_pitch       = yaml["Q_pitch"].as<std::vector<double>>();
        auto R_pitch       = yaml["R_pitch"].as<std::vector<double>>();

        Eigen::Matrix<double, 2, 2> A { { 1, DT }, { 0, 1 } };
        Eigen::Matrix<double, 2, 1> B { { 0 }, { DT } };
        Eigen::Vector2d f { 0, 0 };
        Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
        Eigen::Matrix<double, 1, 1> R(R_pitch.data());
        tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        auto x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        auto x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        auto u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
        auto u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
        tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

        pitch_solver_->settings->max_iter = 10;
    }

    Trajectory trajectory_;
    int shoot_offset_ = 2;

    double fire_thresh_;
    TinySolver* yaw_solver_;
    TinySolver* pitch_solver_;
};

Planner::Planner(const std::string& config_path)
    : pimpl_(std::make_unique<Impl>(config_path)) { }
Planner::~Planner() = default;

auto Planner::SetTrajectory(const Trajectory trajectory) -> void {
    return pimpl_->SetTrajectory(trajectory);
}

auto Planner::Plan(double yaw0) -> const struct Plan {
    return pimpl_->Plan(yaw0);
}

}
