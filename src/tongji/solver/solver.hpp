#pragma once

#include "interfaces/pnp_solver.hpp"
#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::solver {
 class SolverImpl;
class Solver final : public interfaces::IPnpSolver, public interfaces::ITimeStamped {
public:
    explicit Solver(Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
        Eigen::Vector3d t_camera2gimbal);
    ~Solver();

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;

    const std::time_t& GetTimeStamp() const override;

private:
   
    std::unique_ptr<SolverImpl> pimpl_;
};

}