#pragma once

#include "./armor_in_camera.hpp"
#include "./armor_in_image.hpp"
#include "Eigen/src/Geometry/Transform.h"
#include "interfaces/time_stamped.hpp"

namespace world_exe::interfaces {
class IPreDictorUpdatePackage {
public:
    COMBINE_TIME_STAMPED
    const IArmorInCamera& GetArmors() const;
    const Eigen::Affine3d& GetTransform() const;
};
}