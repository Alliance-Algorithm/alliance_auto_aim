#pragma once

#include "./armor_in_camera.hpp"
#include "./armor_in_image.hpp"

namespace world_exe::interfaces {
class IPnpSolver {
public:
    virtual const IArmorInCamera& SolvePnp(const IArmorInImage&);
};
}