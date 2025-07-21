#pragma once

#include "enum/car_id.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
class ICarState {

public:
    virtual const ICarState& Update(const enumeration::CarIDFlag& car_detected);
    virtual const enumeration::CarIDFlag& GetAllowdToFires() const = 0;
};
}