#pragma once

#include "interfaces/pnp_solver.hpp"
#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::solver {

class Solver final : public interfaces::IPnpSolver, public interfaces::ITimeStamped {
public:
    explicit Solver();
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
    class Impl;

    std::unique_ptr<Impl> pimpl_;
};

}