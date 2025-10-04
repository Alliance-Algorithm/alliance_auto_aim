#pragma once

#include <memory>

#include "interfaces/fire_controller.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController();

    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const override;

    const enumeration ::CarIDFlag GetAttackCarId() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}