#pragma once

#include "data/fire_control.hpp"
#include <ctime>

namespace world_exe::interfaces {
class IFireControl {
public:
    virtual const data::FireControl& CalculateTarget(const std::time_t& current) = 0;
};
}