#pragma once

#include "interfaces/fire_controller.hpp"
namespace world_exe::tongji::fire_controller {
class FireControl final : interfaces::IFireControl {

    const data::FireControl CalculateTarget(const std::time_t& time_duration) const override;

    const enumeration::CarIDFlag GetAttackCarId() const override;

    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
