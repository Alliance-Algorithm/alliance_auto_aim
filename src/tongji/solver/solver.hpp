#pragma once

#include "interfaces/pnp_solver.hpp"
#include "interfaces/time_stamped.hpp"

namespace world_exe::tongji::solver {
class SolverImpl;
class Solver final : public interfaces::IPnpSolver, public interfaces::ITimeStamped {
public:
    explicit Solver(const std::string& config_path);
    ~Solver();

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;

    const std::time_t GetTimeStamp() const override;

    Solver(const Solver&)                = delete;
    Solver& operator=(const Solver&)     = delete;
    Solver(Solver&&) noexcept            = default;
    Solver& operator=(Solver&&) noexcept = default;

private:
    std::unique_ptr<SolverImpl> pimpl_;
};

}