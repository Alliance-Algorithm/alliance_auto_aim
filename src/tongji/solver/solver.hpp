#pragma once

#include "interfaces/pnp_solver.hpp"
#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::solver {
class SolverImpl;

struct PnPResultInGimbal {
    Eigen::Vector3d xyz_in_gimbal;
    Eigen::Matrix3d R_armor2gimbal;
    Eigen::Vector3d xyz_in_camera;
    Eigen::Matrix3d R_armor2camera;
};
class Solver final : public interfaces::IPnpSolver, public interfaces::ITimeStamped {
public:
    explicit Solver(const std::string& config_path);
    ~Solver();

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;

    const std::time_t GetTimeStamp() const override;

    auto OptimizeYawByReprojection(const data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
        const double& initial_armor_yaw_in_gimbal) const -> const double;

    Solver(const Solver&)                = delete;
    Solver& operator=(const Solver&)     = delete;
    Solver(Solver&&) noexcept            = default;
    Solver& operator=(Solver&&) noexcept = default;

private:
    std::unique_ptr<SolverImpl> pimpl_;
};

}