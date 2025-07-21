#pragma once

#include "data/fire_control.hpp"
#include "interfaces/car_state.hpp"
#include <ctime>

namespace world_exe::interfaces {
class IFireControl {
public:
    virtual const data::FireControl& CalculateTarget(const std::time_t& current) const = 0;
    virtual const enumeration::CarIDFlag GetAttackCarId() const                        = 0;
};
}