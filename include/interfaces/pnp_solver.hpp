#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"
namespace world_exe::interfaces {
class IPnpSolver {
public:
    virtual std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage>) = 0;
};
}