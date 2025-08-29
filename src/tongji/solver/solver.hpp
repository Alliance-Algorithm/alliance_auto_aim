#pragma once

#include "interfaces/pnp_solver.hpp"

namespace world_exe::tongji::solver {
class Solver final : public interfaces::IPnpSolver, public interfaces::ITimeStamped {
public:
    explicit Solver(const std::string& config_path);
    ~Solver();
    void set_time_point(const std::time_t& time_point);

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;
    const std::time_t& GetTimeStamp() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    std::time_t time_point_;
};

}