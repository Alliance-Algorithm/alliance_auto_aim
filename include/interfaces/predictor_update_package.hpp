#pragma once

#include "./armor_in_camera.hpp"

#include "interfaces/time_stamped.hpp"

namespace world_exe::interfaces {
class IPreDictorUpdatePackage {
public:
    COMBINE_TIME_STAMPED
    virtual const IArmorInCamera& GetArmors() const     = 0;
    virtual const Eigen::Affine3d& GetTransform() const = 0;
};
}